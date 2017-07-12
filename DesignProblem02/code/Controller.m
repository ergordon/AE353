function func = Controller(plot)
% EmilioGordonController   Controller of Gravity-Assisted Robotic Arm
%
%   Controller(ControllerType) aquires the appropriate data from the
%   function 'ControllerType' and outputs the specific graph. For
%   example...Controller(ZeroInput) would display the plots for a
%   controller of Zero Input. 
%     
%   The user must first run at the very least 
%        DesignProblem02('Controller')
%   After the simulation has completed, the user may input any of the 
%   'ControllerType'. The various inputs for Controller() are as follows.
%   
%           ZeroInput : A plot for the Zero-Input open-loop controller will
%                       be generated
%
%           StateFeedback : A plot for the State-feedback closed-loop
%                           controller will be generated
%
%           RefTracking : A plot for the closed-loop controller with
%                         reference tracking will be generated
%
%           StateError : Two plots, one showing Q2 and r w.r.t time and
%                        another containing the steadt state error of the 
%                        system.
%   
%   All entries into Controller() will create plots which are automatically
%   save and appropriatley named in the 'code' folder.
%
%   The controller is set automattically for calculating ReferenceTracking.
%   From there the program works backwords to find ZeroInput and
%   Statefeedback. If you would like to see and publish data for just one
%   of these, you must go into the runControlSystem function and uncomment
%   the apprpriate function and comment the previouse one.Do note however,
%   if you go from ReferenceTracking to ZeroInput, you will not be able to run
%   Controller(RefTracking) since the data does not exist since it never
%   ran.
%
%   Enjoy!
func.init = @initControlSystem;
func.run = @runControlSystem;
end

function [actuators,data] = initControlSystem(sensors,references,parameters,data)
    % Load the equations of motion.
    load('DesignProblem02_EOMs.mat');
    % Parse the equations of motion.
    M = symEOM.M;
    C = symEOM.C;
    N = symEOM.N;
    tau = symEOM.tau;
    % Define symbolic variables that appear in the equations of motion.
    syms q1 q2 v1 v2 tau1 real
    qd = [v1; v2];
    qdd = inv(M)*(-C*qd-N+tau);

    %Equilibrium Points
    equil = [pi 0 0 0 0];

    %Linearization
    Alower = vpa(subs(jacobian(qdd,[q1 q2 v1 v2]),[q1 q2 v1 v2 tau1],equil));
    A = double([0 0 1 0; 0 0 0 1; Alower]);
    Blower = vpa(subs(jacobian(qdd,tau1),[q1 q2 v1 v2 tau1],equil));
    B = double([0; 0; Blower]);
    C = eye(size(A));
    
    %Controllability Check
    Rank = rank(A);
    W = [B A*B A^2*B A^3*B];
    rank(W);
    
    %State Feedback Setup
    x = [sensors.q1; sensors.q2; sensors.v1; sensors.v2];
    x0 = [sensors.q1; sensors.q2; sensors.v1; sensors.v2];
    Q = [100 0 0 0; 0 10000 0 0; 0 0 1 0; 0 0 0 1];
    R = [1];
    K = lqr(A,B,Q,R);
    eig(A);
    eig(A-B*K);
    
    %Reference Tracking Setup
    kRef = -1/(C*inv(A-B*K)*B);
    kRef = kRef(2);
    
    ref = [];
    filename = 'ref.mat';
    save('ref.mat','ref','x','A','B','C','equil','K','kRef','equil','x0');
    
    [actuators,data] = runControlSystem(sensors,references,parameters,data);
end

function [actuators,data] = runControlSystem(sensors,references,parameters,data)
    %%(Un)Comment these accordingly to run your desired function.
      %Zero Input
      %[actuators,data] = zeroInput(sensors,references,parameters,data)
      %State Feedback
      %[actuators,data] = stateFeedback(sensors,references,parameters,data)
      %Reference Tracking
      [actuators,data] = referenceTracking(sensors,references,parameters,data);
end

function [actuators,data] = zeroInput(sensors,references,parameters,data)
    data.q1 = sensors.q1;
    data.q2 = sensors.q2;
    data.v1 = sensors.v1;
    data.v2 = sensors.v2;
    actuators.tau1 = 0;
end

function [actuators,data] = stateFeedback(sensors,references,parameters,data)
    load('ref.mat','K','equil')
    x = [sensors.q1-equil(1); sensors.q2-equil(2); sensors.v1-equil(3); sensors.v2-equil(4)];
    u = -K*x;
    actuators.tau1 = u;
end

function [actuators,data] = referenceTracking(sensors,references,parameters,data)
    load('ref.mat','x','ref','A','B','C','equil','K','kRef','equil','x0')
    x = [sensors.q1-equil(1); sensors.q2-equil(2); sensors.v1-equil(3); sensors.v2-equil(4)];
    r = references.q2;
    u = -K*x+kRef*r;
    ref = [ref r];
    save('ref.mat','ref','x','ref','A','B','C','equil','K','kRef','equil','x0');
    actuators.tau1 = u;
end


