classdef UltrasonicSensor < handle
    properties
        
        arduinoCOMPort;
        arduinoObj;
        
        sensorObj;
        
        distance;
        
        threshold;
        
    end
    methods
        
        function self = UltrasonicSensor(COMPort)
            self.arduinoCOMPort = COMPort;
        end
        
        function Init(self)
            self.arduinoObj = arduino(self.arduinoCOMPort,'Uno','Libraries','Ultrasonic');
            self.sensorObj = ultrasonic(self.arduinoObj,'D2','D3');
        end
        
        function SetThreshold(self, threshold)
            self.threshold = threshold;
        end
        
        function isFree = CheckOcclusion(self)
            self.distance = readDistance(self.sensorObj);
            if self.distance < self.threshold
                isFree = false;
            else
                isFree = true;
            end
        end
    end
end