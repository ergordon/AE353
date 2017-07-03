function [plot1,plot2] = StateFeedback
    load data
    t = processdata.t;

    load('ref.mat','ref','A','B','C','equil','K','kRef','equil','x0')
    
    x = [];
    for i = 1:length(t)
       x = [x expm((A-B*K)*t(i))*x0];
    end
    
    q1 = x(1,:);
    q2 = x(2,:);
    v1 = x(3,:);
    v2 = x(4,:);
    
    subplot(2,1,1);
    plot1 = plot(t,q1,'r-',t,q2,'-','linewidth',3);
    grid on
    set(gca,'fontsize',13);
    legend('Q1','Q2');
    xlabel('Time (seconds)');
    ylabel('Joint Angle (radians)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Joint Angle v. Time')

    subplot(2,1,2);
    plot2=plot(t,v1,'r-',t,v2,'-','linewidth',3);
    grid on
    set(gca,'fontsize',13);
    legend('V1','V2');
    xlabel('Time (seconds)');
    ylabel('Angular Velocity (radians/second)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Joint Angular Velocity v. Time')
    print(gcf,'-dpdf','q1.pdf');
end