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
    end
    
    methods
        function self = LivePBVSWrapper(rosRW)
            %LIVEPBVSWRAPPER 
            self.rosRW = rosRW;
            self.updateFromROS = false;
            self.camImgTopic = '/ppr/robot_cam_img';
            self.camImgSub = rossubscriber(self.camImgTopic, 'sensor_msgs/CompressedImage');
            
            self.camTransform = transl(0,0,0);
            self.camTrTopic = '/ppr/robot_cam_pose';
            self.camTrSub = rossubscriber(self.camTrTopic, 'geometry_msgs/PoseStamped');
            
            self.trQRtoEE = transl(0,0,0.1)*trotx(-pi/2)*trotz(pi/2);
            self.qrInView = false;
            
            self.lambda = 0.5;
            
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
            self.updateFromROS = false;
            self.camImg = img;
        end
        
        function setCamTransform(self, camTransform)
            self.updateFromROS = false;
            self.camTransform = camTransform;
        end
        
        function robotPBVSControl(self)
            if self.updateFromROS
                camTrMsg = self.camTrSub.LatestMessage;
                camImgMsg = self.camImgSub.LatestMessage;
                camImgMsg.Format = 'bgr8; jpeg compressed bgr8';
                
                if ~isempty(camTrMsg) && ~isempty(camImgMsg)
                    self.camTransform = ROSRobotWrapper.PoseStampedToTransform(camTrMsg);
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
                
                % deltaTransform
%                 deltaTransform = inv(poseCurr)*poseDest;
                poseDelta = poseDest - poseCurr;
                deltaTransform = transl(poseDelta(1:3,4));
                Rdot = poseDest(1:3,1:3) - poseCurr(1:3,1:3);
                S = Rdot*poseDest(1:3,1:3)';
                
                % end-effector velocity (P controller)
                xyz = deltaTransform(1:3,4)';
                rpy = [S(3,2) S(1,3) S(2,1)];
                error = [xyz rpy]';
                eeVel = self.lambda*error;
                
                % joint velocities
                Jrobot = self.rosRW.robot.model.jacob0(jointConfig);
                m = sqrt(det(Jrobot*Jrobot'));
                if m < 0.1
                    qdot = pinv(Jrobot'*Jrobot + 0.01*eye(7))*Jrobot'*eeVel;
                else
                    qdot = pinv(Jrobot) * eeVel;                                               % Solve velocitities via RMRC
                end
                
                % check speed limits
                max_qdot = max(qdot(2:7));
                if max_qdot > deg2rad(180)
                    scale = deg2rad(180) / max_qdot;
                    qdot(2:7) = scale*qdot(2:7);
                end
                
                % jog robot
                self.rosRW.jogRobot(qdot);
            end
        end
        
        function analyseImg(self)
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
            catch
                self.qrInView = false;
            end
            self.qrInView = true;
            try
            [newcorners(:,1), newcorners(:,2)] = ...
                transformPointsInverse(tform, self.qrCornersOriginal(:,1),self.qrCornersOriginal(:,2));
            self.trQR = self.centralCam.estpose(self.qrCornersReal, newcorners');
            self.trQR = self.camTransform*self.trQR;
            end
        end
    end
end

