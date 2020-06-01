function trajectory = GenerateTrajectory ( steps )
    if nargin < 1
        steps = 200;
    end
 
    delta = 2*pi/steps;

    x = zeros(3,steps);             % Array for x-y-z trajectory
    theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
    s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
    
    for i=1:steps
        x(1,i) = (1-s(i))*0.35 + s(i)*0.35; % Points in x
        x(2,i) = (1-s(i))*-0.55 + s(i)*0.55; % Points in y
        x(3,i) = 0.5 + 0.2*sin(i*delta); % Points in z
        theta(1,i) = 0;                 % Roll angle
        theta(2,i) = 5*pi/9;            % Pitch angle
        theta(3,i) = 0;                 % Yaw angle
    end
    
    trajectory = [x; theta];
    
end