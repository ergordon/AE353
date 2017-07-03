function [plotSE2,plotSE1] = StateError
    load data
    t = processdata.t;
    q2 = processdata.q2;

    load('ref.mat','ref','A','B','C','equil','K','kRef','equil')
    r = ref(1,:);

    subplot(2,1,1);
    plotSE1 = plot(t,q2,'r-',t,r,'-','linewidth',3);
    set(gca,'fontsize',14);
    grid on
    legend('q2','r');
    xlabel('Time (seconds)');
    ylabel('Joint Angle (radians)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Joint Angle v. Time')

    subplot(2,1,2);
    plotSE2 = plot(t,(q2-r)*57.2958,'-','linewidth',3);
    grid on
    set(gca,'fontsize',14);
    legend('error');
    xlabel('Time (seconds)');
    refline([0 5]);
    ylabel('State Error (degrees)');
    set(gcf,'paperorientation','landscape');
    set(gcf,'paperunits','normalized');
    set(gcf,'paperposition',[0 0 1 1]);
    title('Steady State Error v. Time');
    print(gcf,'-dpdf','StateError120.pdf');
end