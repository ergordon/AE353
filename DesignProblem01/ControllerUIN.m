%
% STEP #1: Rename this file by replacing "UIN" with your UIN number.
%

%
% STEP #2: Rename, but do NOT modify, this function by replacing "UIN" with
% your UIN number.
%

function func = ControllerUIN
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #3: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [actuators,data] = initControlSystem(sensors,references,parameters,data)
actuators.tau1 = 0;
actuators.tau2 = 0;
end

%
% STEP #4: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors,references,parameters,data)
actuators.tau1 = 0;
actuators.tau2 = 0;
end