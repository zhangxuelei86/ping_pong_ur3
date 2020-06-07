classdef GameController < handle
    
    properties
        id;
        joystick;
    end
    
    methods
        function self = GameController(id)
            
            self.id = id;
            self.joystick = vrjoystick(id);
            
            %% For debug purpose, joystick info is printed
            
            joystickInfo = caps(self.joystick);
            fprintf('This joystick has:\n');
            fprintf(' - %i buttons\n',joystickInfo.Buttons);
            fprintf(' - %i axes\n', joystickInfo.Axes);
            
        end
        
        function GetContollerCommands(self)
            
            [axes, buttons, povs] = read(joy);
            
        end
    end
end