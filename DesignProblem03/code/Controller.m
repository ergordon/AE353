function func = Controller
% INTERFACE
%
%   sensors
%       .theta      (pitch angle)
%       .phi        (elevator angle)
%
%   references
%       .theta      (reference pitch angle)
%
%   parameters
%       .tStep      (time step)
%       .phidotMax  (maximum elevator angular velocity)  
%       .symEOM     (nonlinear EOMs in symbolic form)
%       .numEOM     (nonlinear EOMs in numeric form)
%
%   data
%       .whatever   (yours to define - put whatever you want into "data")
%
%   actuators
%       .phidot     (elevator angular velocity)

% Do not modify this function.
func.init = @initControlSystem;
func.run = @runControlSystem;

end

%
% STEP #1: Modify, but do NOT rename, this function. It is called once,
% before the simulation loop starts.
%

function [actuators,data] = initControlSystem(sensors,references,parameters,data)

%
% Here is a good place to initialize things like gains...
%
%   data.K = [1 2 3 4];
%
% ...or like the integral of error in reference tracking...
%
%   data.v = 0;
%
% Load the equations of motion.
load('DesignProblem03_EOMs.mat');
% Parse the equations of motion.
f = symEOM.f;
% Define symbolic variables that appear in the equations of motion.
syms theta phi xdot zdot thetadot phidot
%Choose State
fsym=[f;thetadot;phidot];

%Equilibrium Points
%GetEquilibriumPoint(f,theta,phi,xdot,zdot)
[thetae,phie,xdote,zdote] = GetEquilibriumPoint(f,0,0,6,0);%(f,0,0,6,0);
%pause
thetadote = 0;
phidote = 0;
xhat = [0; 0; 0; 0; 0];
eqstate = [xdote;zdote;thetadote;thetae;phie]
eqinput = [phidote];
state = [xdot; zdot; thetadot; theta; phi];
input = [phidot];
%Linearization
A = double([vpa(subs(jacobian(fsym,state),[state; input],[eqstate; eqinput]))]);
B = double(vpa(subs(jacobian(fsym,input),[state; input],[eqstate; eqinput])));
C = [0 0 0 1 0;0 0 0 0 1];

%Verfies System is Controllable
rank(ctrb(A,B))-length(A);

%Define Gains Controller
%Qc = [100000 0 0 0 0;0 1000 0 0 0;0 0 10000 0 0;0 0 0 100000 0;0 0 0 0 10000];
%Qc = [1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 10 0;0 0 0 0 1];
%Rc = 1;%1/100
Qc = eye(5)
Rc = eye(1)
K = lqr(A,B,Qc,Rc);

%Verifies if Controller is Asymptoticly Stable
eig(A-B*K);

%Verifies System is Observable
rank(obsv(A,C))-length(A);

%Define Gains Observer
 Qo = eye(2)%[1 0; 0 1];
 Ro = eye(5);%[1 0 0 0 0;0 1 0 0 0;0 0 1 0 0;0 0 0 1 0;0 0 0 0 1];
%Qo = [400 0; 0 400];
%Ro = [1/9 0 0 0 0;0 1/9 0 0 0;0 0 1 0 0;0 0 0 1 0;0 0 0 0 1/9];
L = lqr(A',C',inv(Ro),inv(Qo))';
%Verifies if Observer is Asymptoticly Stable
eig(A-L*C)


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

%%This is stuff recently added
%data.zhat = 0;

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

%zhat = data.zhat;%%This was recently added
xhat = data.xhat;
eqstate = data.eqstate;
eqinput = data.eqinput;
%WORKING CODE. SAVE FOR SCREW UPS
%%{
theta = sensors.theta;
phi = sensors.phi;

output = [theta; phi];

y = output - C*[eqstate];%error
u =-K*xhat;%input
dxhat = A*xhat+B*u-L*(C*xhat-y);%this is obersever. State Estimate
data.xhat = xhat + h*dxhat; % Next step for state estimate.

%actuators.phidot = 0;
actuators.phidot = u+eqinput;%input taking into account eqinput
%}
%WRONG CODE
%{
theta = sensors.theta;
phi = sensors.phi;

output = [theta; phi];

cRef = [0 0 0 1 0];
kRef = -1/(cRef*inv(A-B*K)*B)
r = 0.1

y = output - C*[eqstate];%error
u =-K*xhat - kRef*r%input

dxhat = A*xhat+B*u-L*(C*xhat-y);%this is obersever. State Estimate
data.xhat = xhat + h*dxhat;% Next step for state estimate.

actuators.phidot = u+eqinput;%input taking into account eqinput
%}

end

% INPUTS:
% f is a symbolic description of the nonlinear EOMs
%   (theta, phi, xdot, zdot) is a guess at the equilibrium point
% OUTPUTS:
%   (theta, phi, xdot, zdot) is an equilibrium point near the guess
function [theta,phi,xdot,zdot] = GetEquilibriumPoint(f,theta,phi,xdot,zdot)
% Initial guess
x0 = [theta;phi;xdot;zdot];
% Symbolic states
syms theta phi xdot zdot thetadot real
% Symbolic inputs
syms phidot real
% Symbolic EOMs with thetadot=0 and phidot=0
g = subs(f,[thetadot,phidot],[0,0]);
% Numeric EOMs
vars = [theta;phi;xdot;zdot];
g = matlabFunction(g,'Vars',{vars});
% Find solution
xe = fsolve(g,x0);
% Parse solution
theta = xe(1);
phi = xe(2);
xdot = xe(3);
zdot = xe(4);
end