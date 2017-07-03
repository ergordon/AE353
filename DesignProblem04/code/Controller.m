function func = Controller
% EmilioGordonController   Controller of Segway Robot
%
%   
%   The user must first run at the very least 
%        DesignProblem04('Controller')
%   After the simulation has completed, the user may input any of the 
%   'ControllerType'. The various inputs for Controller() are as follows.
%   
%          Controller : Controller simply runs a closed loop system.
%
%          Controller2 : Controller runs a controller implementing loop
%          efficiency.
%
%   Enjoy!
func.init = @initControlSystem;
func.run = @runControlSystem;
end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [actuators,data] = initControlSystem(sensors,references,parameters,data)
% Load the equations of motion.
load('DesignProblem04_EOMs.mat');
% Parse the equations of motion.
f = symEOM.f;
% Define symbolic variables that appear in the equations of motion.
syms phi phidot v w tauR tauL elateral eheading vroad wroad

%0 ve we 0 0 0 <- always in equillibrium....

%Choose State
vroad = 10
wroad = vroad/(sensors.r_road)
fsym=[f;phidot; -v*sin(eheading); w-((v*cos(eheading))/(vroad+wroad*elateral))*wroad];

%Equilibrium Points
xhat = [0; 0; 0; 0; 0; 0];
eqstate = [0; 1.5; 0; 0; 0; 0];
eqinput = [0; 0];
state = [phidot; v; w; phi; elateral;eheading];
input = [tauR; tauL];
%Linearization
A = double([vpa(subs(jacobian(fsym,state),[state; input],[eqstate; eqinput]))]);
B = double(vpa(subs(jacobian(fsym,input),[state; input],[eqstate; eqinput])));
C = [0 1 0 0 0 0; 
    0 0 1 0 0 0; 
    0 0 0 1 0 0; 
    0 0 0 0 1 0; 
    0 0 0 0 0 1];

% Design an optimal controller with reference tracking
%Define Gains Controller
disp('controller')
Qc = 500*[1 0 0 0 0 0;0 15 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 100 0;0 0 0 0 0 100];
Rc = [1 0;0 1];
K = lqr(A,B,Qc,Rc);
disp('K')
mat2str(K)
disp('stability')
eig(A-B*K)
disp('controllerability')
rank(ctrb(A,B))-length(A)

%Define Gains Observer
Ro = 10*[10 0 0 0 0 0;0 1 0 0 0 0;0 0 10 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
Qo = [1 0 0 0 0;0 1 0 0 0;0 0 10 0 0;0 0 0 10 0;0 0 0 0 10];
L = lqr(A',C',inv(Ro),inv(Qo))';
eig(A-L*C)
rank(obsv(A,C))-length(A)

%Saving Variables
data.A = A;
data.B = B;
data.C = C;
data.K = K;
data.L = L;
data.h = parameters.tStep;
data.eqstate = eqstate;
data.eqinput = eqinput;
data.xhat = xhat;

[actuators,data] = runControlSystem(sensors,references,parameters,data);
end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors,references,parameters,data)
A = data.A;
B = data.B;
C = data.C;
K = data.K;
L = data.L;
h = data.h;
xhat = data.xhat;
eqstate = data.eqstate;
eqinput = data.eqinput;

output = [sensors.v; sensors.w; sensors.phi; sensors.e_lateral;sensors.e_heading];

y = output - C*[eqstate];%error
u =-K*xhat;%input
dxhat = A*xhat+B*u-L*(C*xhat-y);%this is obersever. State Estimate
data.xhat = xhat + h*dxhat; % Next step for state estimate.



actuators.tauR = u(1);
actuators.tauL = u(2);
end