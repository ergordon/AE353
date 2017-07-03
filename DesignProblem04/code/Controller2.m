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
load('DesignProblem04_EOMs.mat');
syms phi phidot v w tauR tauL elateral eheading vroad wroad
f = symEOM.f;

vroad = 10
wroad = vroad/(sensors.r_road*0.5)
fsym=[f;phidot; -v*sin(eheading); w-((v*cos(eheading))/(vroad+wroad*elateral))*wroad];

syms V W TAUR TAUL real
%Equilibrium Points
xhat = [0; 0; 0; 0; 0; 0];
state = [phidot; v; w; phi; elateral;eheading];
input = [tauR; tauL];
%Linearization
A = subs(jacobian(fsym,state),[state; input],[[0.015; 2; W; 0; 0; 0]; [0; 0]])
B = subs(jacobian(fsym,input),[state; input],[[0.015; 2; W; 0; 0; 0]; [0; 0]])
C = [0 1 0 0 0 0; 0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;]


%Saving Variables
data.funcA = matlabFunction(A)
data.funcB = matlabFunction(B);
data.C = C;
data.h = parameters.tStep;
data.xhat = xhat;

[actuators,data] = runControlSystem(sensors,references,parameters,data);
end

%
% STEP #2: Modify, but do NOT rename, this function. It is called every
% time through the simulation loop.
%

function [actuators,data] = runControlSystem(sensors,references,parameters,data)
V = sensors.v;
W = sensors.w;

eqstate = [0.015; 2; W; 0; 0; 0]

A = data.funcA(W);
B = data.funcB();

C = data.C;
h = data.h;
xhat = data.xhat;
%state = [phidot; v; w; phi; elateral;eheading];

% Design an optimal controller with reference tracking
%Define Gains Controller
Qc = 500*[1 0 0 0 0 0;0 15 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
Rc = [1 0;0 1];
K = lqr(A,B,Qc,Rc);

%Define Gains Observer
Ro = 10*[100 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1];
Qo = [1 0 0 0 0;0 1 0 0 0;0 0 100 0 0;0 0 0 100 0;0 0 0 0 100];
L = lqr(A',C',inv(Ro),inv(Qo))';

output = [sensors.v; sensors.w; sensors.phi; sensors.e_lateral;sensors.e_heading];
 
y = output - C*[eqstate];%error
u =-K*xhat;%input
dxhat = A*xhat+B*u-L*(C*xhat-y);%this is obersever. State Estimate
data.xhat = xhat + h*dxhat; % Next step for state estimate.

actuators.tauR = u(1);
actuators.tauL = u(2);

end