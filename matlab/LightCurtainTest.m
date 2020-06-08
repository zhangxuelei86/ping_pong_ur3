%% Test script for testing intersection with the Light Curtains
function LightCurtainTest
    close all;
    clear all;
    clc;

    %% Create sources and then light curtain group
    sources = [];
    numSources = 5;
    
    for i = 1:numSources
        sources(i).origin = rand(1,3) .* randi([-1 1],1,3);
        sources(i).yaw = rand * 360;
        sources(i).angDiff = 5;
    end
    
    lightCurt = LightCurtain(sources,true);
    
    %% Test points of breach in Light curtain
    breached = lightCurt.CheckBreach([0.4 -1.1 0]);

end