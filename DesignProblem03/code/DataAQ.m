% Number of flights
nFlights = 1000;

%Vars
Xf = zeros(1,nFlights);
xx = []
tt = []
% Loop over each flight
tic
for i=1:nFlights
% Run simulation without graphics and save data
DesignProblem03('Controller','datafile','data.mat','display',false,'initial',[0;2;0;0.1;10;0;0]);
% Load data
load('data.mat');
% Get t and x
t = processdata.t;
x = processdata.x;

xx = [xx x];
tt = [tt t];
% Do analysis...
%
i
Xf(i) = x(end);
end
toc
histogram(Xf)
    set(gca,'fontsize',14);
    xlabel('Distance (x)');
    ylabel('Frequency');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Frequency Distribution of 1000 Simulated Flights')
    print(gcf,'-dpdf','flights.pdf');
mean(Xf)
median(Xf)

m = matfile(xFlight1,'Writable',isWritable)
save(xFlight,Xf)