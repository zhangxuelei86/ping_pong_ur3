classdef LivePBVSWrapper < handle
    %LIVEPBVSWRAPPER Takes in the ROSRobotWrapper class, can subscribes to
    %an image topic, or set an image manually (for testing)
    
    properties
        rosRW;
        
        camImg;
        updateFromROS;
        camImgTopic;
        camImgSub;
        
        centralCam;
        camFocal;
        camRes;
        camTransform;
        camTrTopic;
        camTrSub;
        camTrUpdated;
        
        qrImgOriginal;
        ptsOriginal;
        featuresOriginal;
        validPtsOriginal;
        qrCornersOriginal;
        
        qrRealSize;
        qrCornersReal;
        
        trQRtoEE;
        PBVSControlTimer;
        qrInView;
        trQR;
        lambda;
        maxManipulability;
        jointSpeedLimit;
    end
    
    methods
        function self = LivePBVSWrapper(rosRW)
            %LIVEPBVSWRAPPER Contructor - initialise fixed properties such
            %as EE transform from QR code, P controller gain and maximum
            %manipulability value for singularity avoidance
            
            self.rosRW = rosRW;
            self.updateFromROS = false;
            self.camImgTopic = '/ppr/robot_cam_img';
            self.camImgSub = rossubscriber(self.camImgTopic, 'sensor_msgs/CompressedImage');
            
            self.camTransform = transl(0,0,0);
            self.camTrTopic = '/ppr/robot_cam_pose';
            self.camTrSub = rossubscriber(self.camTrTopic, 'geometry_msgs/PoseStamped');
            
            self.trQRtoEE = transl(0,0,0.1)*trotx(-pi/2)*trotz(pi/2);
            self.qrInView = false;
            
            self.lambda = 1.5;
            self.maxManipulability = 0.1;
            self.jointSpeedLimit = deg2rad(180);
            self.camTrUpdated = false;
            
            self.PBVSControlTimer = timer('StartDelay', 0, 'Period', 0.1, ...
                    'TasksToExecute', Inf, 'ExecutionMode', 'fixedSpacing');
            self.PBVSControlTimer.TimerFcn = @(obj, event)robotPBVSControl(self);
        end
        
        function delete(self)
            %LIVEPBVSWRAPPER Destructor
            stop(self.PBVSControlTimer); % just to be sure
        end
        
        function init(self, camFocal, camRes)
            %INIT Initialise cameras and other necessary objects
            
            self.camFocal = camFocal;
            self.camRes = camRes;
            self.centralCam = CentralCamera('name', 'PBVScam', ...
                                            'focal', self.camFocal, ...
                                            'pixel', 10e-5, ...
                                            'resolution', self.camRes);
                                        
            self.qrImgOriginal = rgb2gray(imread('retreat_code.png'));
            
            % Extract features from the QR reference image
            self.ptsOriginal  = detectSURFFeatures(self.qrImgOriginal);
            [self.featuresOriginal,self.validPtsOriginal] = ...
                extractFeatures(self.qrImgOriginal,self.ptsOriginal);
            [width, height,~] = size(self.qrImgOriginal);
            self.qrCornersOriginal = [0,0;height,0;height,width;0,width];
            
            % Dimensions of the real QR code
            self.qrRealSize = 0.2; % 20 cm
            realCorner1 = [-0.5*self.qrRealSize; 0.5*self.qrRealSize; 0];
            realCorner2 = [0.5*self.qrRealSize; 0.5*self.qrRealSize; 0];
            realCorner3 = [0.5*self.qrRealSize; -0.5*self.qrRealSize; 0];
            realCorner4 = [-0.5*self.qrRealSize; -0.5*self.qrRealSize; 0];
            self.qrCornersReal = [realCorner1 realCorner2 realCorner3 realCorner4];
        end
        
        function enableROSUpdate(self, status)
            self.updateFromROS = status;
        end
        
        function startPBVS(self)
            try start(self.PBVSControlTimer);
            catch
                disp("Warning: PBVS already started");
            end
        end
        
        function stopPBVS(self)
            stop(self.PBVSControlTimer);
        end
        
        function setImage(self, img)
            %SETIMAGE Manually sets the image (disables ROS update)
            self.updateFromROS = false;
            self.camImg = img;
        end
        
        function setCamTransform(self, camTransform)
            %SETIMAGE Manually sets the camera transform (disables ROS update)
            self.updateFromROS = false;
            self.camTransform = camTransform;
        end
        
        function updateCamTransform(self)
            %UPDATECAMTRANSFORM Update the camera transform from ROS (does
            %this once only)
            camTrMsg = receive(self.camTrSub, 1);
            if ~isempty(camTrMsg)
                self.camTransform = ROSRobotWrapper.PoseStampedToTransform(camTrMsg);
                self.camTrUpdated = true;
            end
        end
        
        function qdot = GetQDotFromEEVel(self, eeVel)
            %GETQDOTFROMEEVEL Gets the joint velocities matrix from the
            %end-effector's desired velocity, takes maximum joint velocity
            %into account

            jointConfig = self.rosRW.robot.model.getpos();
            Jrobot = self.rosRW.robot.model.jacob0(jointConfig);
            m = sqrt(det(Jrobot*Jrobot'));
            if m < self.maxManipulability
                qdot = pinv(Jrobot'*Jrobot + 0.01*eye(7))*Jrobot'*eeVel;
            else
                qdot = pinv(Jrobot) * eeVel;
            end

            % check speed limits
            max_qdot = max(qdot(2:7));
            if max_qdot > self.jointSpeedLimit
                scale = self.jointSpeedLimit / max_qdot;
                qdot(2:7) = scale*qdot(2:7);
            end
        end
        
        function robotPBVSControl(self)
            %ROBOTPBVSCONTROL When called, control the arm velocity by
            %analysing the camera image, find and estimate the pose of the
            %QR code, and make the end-effector go to the desired pose.
            %This function should be called continously in a while loop to
            %enables the 'safety' feature that retreats the robot
            
            if self.updateFromROS
                if ~ self.camTrUpdated
                    self.updateCamTransform();
                end
                
                camImgMsg = receive(self.camImgSub, 1);
                camImgMsg.Format = 'bgr8; jpeg compressed bgr8';
                if ~isempty(camImgMsg)
                    self.camImg = readImage(camImgMsg);
                end
            end
            self.analyseImg();
            % IF QR CODE IS VISIBLE
            if self.qrInView
                % gets current robot end-effector pose
                jointConfig = self.rosRW.robot.model.getpos();
                poseCurr = self.rosRW.robot.model.fkine(jointConfig);
                
                % desired end-effector pose
                poseDest = self.trQR*self.trQRtoEE;
                
                % Find xyz rpy from current -> destination
                poseDelta = poseDest - poseCurr;
                Rdot = poseDelta(1:3,1:3);
                S = Rdot*poseDest(1:3,1:3)';
                xyz = poseDelta(1:3,4)';
                rpy = [S(3,2) S(1,3) S(2,1)];
                error = [xyz rpy]';
                
                % end-effector velocity (P controller)
                eeVel = self.lambda*error;
                
                qdot = self.GetQDotFromEEVel(eeVel);

                % jog robot
                self.rosRW.jogRobot(qdot);
            end
        end
        
        function analyseImg(self)
            %ANALYSEIMG Analyse the camera image, using SURF Features to
            %detect the QR code and functions from Peter Corke's Machine
            %Vision Toolbox to estimate the QR code's pose in world frame
            
            if isempty(self.camImg)
                return;
            end
            distorted = rgb2gray(self.camImg);
            ptsDistorted = detectSURFFeatures(distorted);
            [featuresDistorted,validPtsDistorted] = extractFeatures(distorted,ptsDistorted);
            index_pairs = matchFeatures(self.featuresOriginal,featuresDistorted);

            matchedPtsOriginal  = self.validPtsOriginal(index_pairs(:,1));
            matchedPtsDistorted = validPtsDistorted(index_pairs(:,2));

            try
                [tform,~,~] = estimateGeometricTransform(matchedPtsDistorted,matchedPtsOriginal, 'projective');
                [newcorners(:,1), newcorners(:,2)] = ...
                    transformPointsInverse(tform, self.qrCornersOriginal(:,1),self.qrCornersOriginal(:,2));
                self.trQR = self.centralCam.estpose(self.qrCornersReal, newcorners');
                self.trQR = self.camTransform*self.trQR;
                self.qrInView = true;
            catch
                self.qrInView = false;
            end
        end
    end
end

