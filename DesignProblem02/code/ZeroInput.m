function [plot1,plot2] = ZeroInput
    load data
    t = processdata.t;
    q1 = processdata.q1;
    q2 = processdata.q2;
    v1 = processdata.v1;
    v2 = processdata.v2;
    
    subplot(2,1,1);
    plot1 = plot(t,q1,'r-',t,q2,'-','linewidth',3);
    grid on
    set(gca,'fontsize',14);
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
    set(gca,'fontsize',14);
    legend('V1','V2');
    xlabel('Time (seconds)');
    ylabel('Angular Velocity (radians/second)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Joint Angular Velocity v. Time')
    print(gcf,'-dpdf','zisStable.pdf');
end