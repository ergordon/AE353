function [plot1,plot2] = RefTracking
    load data
    t = processdata.t;
    q1 = processdata.q1;
    q2 = processdata.q2;
    v1 = processdata.v1;
    v2 = processdata.v2;
    
    load('ref.mat','ref','A','B','C','equil','K','kRef','equil')
    r = ref(1,:);
    
    subplot(2,1,1);
    plot1 = plot(t,q1,'r-',t,q2,'-','linewidth',3);
    grid on
    set(gca,'fontsize',14);
    legend('q1','q2');
    xlabel('Time (seconds)');
    ylabel('Joint Angle (radians)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Joint Angle v. Time')

    subplot(2,1,2);
    plot2 = plot(t,q2,'r-',t,r,'-','linewidth',3);
    grid on
    set(gca,'fontsize',14);
    legend('q2','ref');
    xlabel('Time (seconds)');
    ylabel('Joint Angle (radians)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Joint Angle v. Time')
    
    print(gcf,'-dpdf','RefTracking2.pdf');
end