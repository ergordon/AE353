% Controller675895697 - Emilio Gordon
% User Guide
% 1. Run the command
%    DesignProblem01('Controller675895697','datafile','data.mat','diagnostics',true, 'tstop',60)
% 2. Allow the simulation to finish running.
% 3. While the simulation is running, decide wether
%    you want to compute ZeroInput, StateFeedback or 
%    Reference Tracking
% 4. Once simulation is complete. Uncomment your chosen 
%    computation and click run. The approproate plots will
%    appear.
%
%   *NOTE* Since the command in step 1 runs the runControlSystem
%   function, the angular velocities in the simulation will be with
%   reference tracking implemented to the input. Therefore,
%   ZeroInput will not be accurate. Neither will State 
%   Feedback but it will still appear close.
%   As a result, PlotReferenceTracking is 
%   uncommented by default

function func = Controller675895697
func.init = @initControlSystem;
func.run = @runControlSystem;
% PlotZeroInput
% PlotStateFeedback
 PlotReferenceTracking
end

function [actuators,data] = initControlSystem(sensors,references,parameters,data)

w0 = [sensors.w1; sensors.w2; sensors.w3];
%Create Global Variables
setGlobalw0(w0);
setGlobalJ1(parameters.J1);
setGlobalJ2(parameters.J2);
setGlobalJ3(parameters.J3);

actuators.tau1 = 0;
actuators.tau2 = 0;
end

function [actuators,data] = runControlSystem(sensors,references,parameters,data)
t = sensors.t;
w = [sensors.w1 sensors.w2 sensors.w3];
%Get Global Variables
w0 = getGlobalw0;
J1 = getGlobalJ1;
J2 = getGlobalJ2;
J3 = getGlobalJ3;
%Define Equilibrium
we = [0; 7.292115*10^-5; 0];
%Define Model
A = [0 0 ((J2-J3)/J1)*we(2);...
    0 0 0;...
    ((J1-J2)/J3)*we(2) 0 0];
B = [1/J1 0;...
    0 1/J2;...
    0 0];

C = eye(3);
D = [0];

%Make Up K
K = [1 0 -1;-0.5 0.01 0]
%Eigenvector for Statefeedback
[V,F1] = eig(A-B*K);
 FStateFeedback = F1
%Eigenvector for ZeroInput
[V,F2] = eig(A);
FZeroInput = F2

x = [sensors.w1; sensors.w2; sensors.w3];

%Define kRef
kRefBase=-C*inv(A-B*K)*B;
kRef = inv([0 kRefBase(2,2); kRefBase(3,1) 0]);
%we have two kRefs because we have two inputs!
kRef1 = kRef(1,2);
kRef2 = kRef(2,1);

r = [0;7.292115*10^-5];
u = -K*x + kRef2*r;

actuators.tau1 = u(1);
actuators.tau2 = u(2);
end

function setGlobalw0(x)
global winit
winit = x;
end

function w0 = getGlobalw0
global winit
w0 = winit;
end

function setGlobalJ1(x1)
global winit1
winit1 = x1;
end

function J1 = getGlobalJ1
global winit1
J1 = winit1;
end

function setGlobalJ2(x2)
global winit2
winit2 = x2;
end

function J2 = getGlobalJ2
global winit2
J2 = winit2;
end

function setGlobalJ3(x3)
global winit3
winit3 = x3;
end

function J3 = getGlobalJ3
global winit3
J3 = winit3;
end
%ZeroInput
function zeroInput = PlotZeroInput()
    load data
    t = processdata.t;
    w = processdata.w_01in1;
    
    w0 = getGlobalw0;
    J1 = getGlobalJ1;
    J2 = getGlobalJ2;
    J3 = getGlobalJ3;
    we = [0; 7.292115*10^-5; 0];
    
    %Define Model
    A = [0 0 ((J2-J3)/J1)*we(2); 0 0 0; ((J1-J2)/J3)*we(2) 0 0];
    B = [1/J1 0; 0 1/J2; 0 0];
    C = eye(3);
    D = [0];
    
    [V,F] = eig(A);
    FZeroInput = F
    
    % Simulate linear system (analytical solution)
     for i = 1:length(t)
         x(:,i) = (expm(A*t(i))*w0);
     end
     
    % Plot results
    subplot(3,1,1);
plot(t,x(1,:),'r-',t,w(1,:),'-','linewidth',2);
set(gca,'fontsize',12);
legend('Equilibrium','Real (Simulated) Values');
xlabel('Time (seconds)');
set(gcf,'paperorientation','landscape');
set(gcf,'paperunits','normalized');
set(gcf,'paperposition',[0 0 1 1]);
title('Angular Velocity 1 v. Time')

subplot(3,1,2);
plot(t,x(2,:)+we(2),'r-',t,w(2,:),'-','linewidth',2);
set(gca,'fontsize',12);
xlabel('Time (seconds)');
ylabel('Angular Velocity (radians/second)');
set(gcf,'paperorientation','landscape');
set(gcf,'paperunits','normalized');
set(gcf,'paperposition',[0 0 1 1]);
title('Angular Velocity 2 v. Time')

subplot(3,1,3);
plot(t,x(3,:),'r-',t,w(3,:),'-','linewidth',2);
set(gca,'fontsize',12);
xlabel('Time (seconds)');
set(gcf,'paperorientation','landscape');
set(gcf,'paperunits','normalized');
set(gcf,'paperposition',[0 0 1 1]);
title('Angular Velocity 3 v. Time')
print(gcf,'-dpdf','rt.pdf');
end
%StateFeedback
function stateFeedback = PlotStateFeedback
    load data
    t = processdata.t;
    w = processdata.w_01in1;
    
    w0 = getGlobalw0
    J1 = getGlobalJ1;
    J2 = getGlobalJ2;
    J3 = getGlobalJ3;
    
    we = [0; 7.292115*10^-5; 0];

    A = [0 0 ((J2-J3)/J1)*we(2);...
    0 0 0;...
    ((J1-J2)/J3)*we(2) 0 0];

    B = [1/J1 0;...
    0 1/J2;...
    0 0];

    C = eye(3);
    D = [0];

    %Make Up K
    %K = [1 0 -1;-1 1 0]
    K = [7 0 -1;-1 3 0];
    % Simulate linear system (analytical solution)
    E = A-B*K;
    [V,F]=eig(E);
    FStateFeedback = F
    for i = 1:length(t)
        x(:,i) = expm((A-B*K)*t(i))*w0;
    end
    % Plot results
    %plot(t,w,'-','linewidth',2);
    subplot(3,1,1);
    plot(t,x(1,:),'r--',t,w(1,:),'-','linewidth',2);
    set(gca,'fontsize',12);
    legend('Linearized Model Values','Real (Simulated) Values');
    xlabel('Time (seconds)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Angular Velocity 1 v. Time')
    
    subplot(3,1,2);
    plot(t,x(2,:)+we(2),'r--',t,w(2,:),'-','linewidth',2);
    %axis([0.00 0.001 0 1])
    set(gca,'fontsize',12);
    xlabel('Time (seconds)');
    ylabel('Angular Velocity (radians/second)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Angular Velocity 2 v. Time')
    
    subplot(3,1,3);
    plot(t,x(3,:),'r--',t,w(3,:),'-','linewidth',2);
    set(gca,'fontsize',12);
    xlabel('Time (seconds)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Angular Velocity 3 v. Time')
    print(gcf,'-dpdf','wall.pdf');
end
%Reference Tracking
function referenceTracking = PlotReferenceTracking
load data
t = processdata.t;
w = processdata.w_01in1;

w0 = getGlobalw0
J1 = getGlobalJ1;
J2 = getGlobalJ2;
J3 = getGlobalJ3;

we = [0; 7.292115*10^-5; 0];

A = [0 0 ((J2-J3)/J1)*we(2);...
    0 0 0;...
    ((J1-J2)/J3)*we(2) 0 0];

B = [1/J1 0;...
    0 1/J2;...
    0 0];

C = eye(3);
D = [0];

%Make Up K
% K = [1 0 -1;-1 1 0]
% K = [7 0 -1;-1 3 0]
K = [1 0 -1;-0.5 0.01 0]
%Define kRef
kRefBase=-C*inv(A-B*K)*B;
kRef = inv([0 kRefBase(2,2); kRefBase(3,1) 0]);
%we have two kRefs because we have two inputs!
kRef1 = kRef(1,2);
kRef2 = kRef(2,1);
r = [0;7.292115*10^-5];
% Simulate linear system (analytical solution)
for i = 1:length(t)
    x(:,i) = -C*inv(A-B*K)*B*kRef2*r;
end
% Plot results
subplot(3,1,1);
plot(t,x(1,:),'r-',t,w(1,:),'-','linewidth',2);
set(gca,'fontsize',12);
legend('Equilibrium','Real (Simulated) Values');
xlabel('Time (seconds)');
set(gcf,'paperorientation','landscape');
set(gcf,'paperunits','normalized');
set(gcf,'paperposition',[0 0 1 1]);
title('Angular Velocity 1 v. Time')

subplot(3,1,2);
plot(t,x(2,:)+we(2),'r-',t,w(2,:),'-','linewidth',2);
set(gca,'fontsize',12);
xlabel('Time (seconds)');
ylabel('Angular Velocity (radians/second)');
set(gcf,'paperorientation','landscape');
set(gcf,'paperunits','normalized');
set(gcf,'paperposition',[0 0 1 1]);
title('Angular Velocity 2 v. Time')

subplot(3,1,3);
plot(t,x(3,:),'r-',t,w(3,:),'-','linewidth',2);
set(gca,'fontsize',12);
xlabel('Time (seconds)');
set(gcf,'paperorientation','landscape');
set(gcf,'paperunits','normalized');
set(gcf,'paperposition',[0 0 1 1]);
title('Angular Velocity 3 v. Time')
print(gcf,'-dpdf','rt.pdf');

end
