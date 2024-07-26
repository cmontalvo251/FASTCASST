close all
%%%States
ylabels = {'x','y','z','\phi','\theta','\psi','u','v','w','p','q','r'}';
linS = {'-','--'};
for idx = 1:12
    figure();
    hold on
    for idn = 1:nCrafts
        state(idx,:) = metaState(idx,idn,:);
        if idx > 3 && idx < 7
            f = 180/pi;
        else
            f = 1;
        end
        plot(time,state(idx,:)*f,'k','linestyle',linS{idn},'LineWidth',3)
    end
    xlabel('Time (sec)')
    ylabel(ylabels{idx})
    legend('Left Aircraft','Right Aircraft')
end
%%%Forces and Moments
%force_aeroB,force_gravB,force_jointB,torque_aeroB,torque_jointB]
forcelabels = {'FxAero (N)','FyAero (N)','FzAero (N)'};
for idx = 1:3
    figure()
    hold on
    for idn = 1:nCrafts
        plot(time,force_aeroB(idx,:,idn),'k','linestyle',linS{idn},'LineWidth',3)
    end
    xlabel('Time (sec)')
    ylabel(forcelabels{idx})
    legend('Left Aircraft','Right Aircraft')
end
forcelabels = {'FxGrav (N)','FyGrav (N)','FzGrav (N)'};
for idx = 1:3
    figure()
    hold on
    for idn = 1:nCrafts
        plot(time,force_gravB(idx,:,idn),'k','linestyle',linS{idn},'LineWidth',3)
    end
    xlabel('Time (sec)')
    ylabel(forcelabels{idx})
    legend('Left Aircraft','Right Aircraft')
end
forcelabels = {'FxJoint (N)','FyJoint (N)','FzJoint (N)'};
for idx = 1:3
    figure()
    hold on
    for idn = 1:nCrafts
        plot(time,force_jointB(idx,:,idn),'k','linestyle',linS{idn},'LineWidth',3)
    end
    xlabel('Time (sec)')
    ylabel(forcelabels{idx})
    legend('Left Aircraft','Right Aircraft')
end

forcelabels = {'FxTotal (N)','FyTotal (N)','FzTotal (N)'};
for idx = 1:3
    figure()
    hold on
    for idn = 1:nCrafts
        plot(time,force_jointB(idx,:,idn)+force_aeroB(idx,:,idn)+force_gravB(idx,:,idn),'k','linestyle',linS{idn},'LineWidth',3)
    end
    xlabel('Time (sec)')
    ylabel(forcelabels{idx})
    legend('Left Aircraft','Right Aircraft')
end

torquelabels = {'TxAero (N-m)','TyAero (N-m)','TzAero (N-m)'};
for idx = 1:3
    figure()
    hold on
    for idn = 1:nCrafts
        plot(time,torque_aeroB(idx,:,idn),'k','linestyle',linS{idn},'LineWidth',3)
    end
    xlabel('Time (sec)')
    ylabel(torquelabels{idx})
    legend('Left Aircraft','Right Aircraft')
end
torquelabels = {'TxJoint (N-m)','TyJoint (N-m)','TzJoint (N-m)'};
for idx = 1:3
    figure()
    hold on
    for idn = 1:nCrafts
        plot(time,torque_jointB(idx,:,idn),'k','linestyle',linS{idn},'LineWidth',3)
    end
    xlabel('Time (sec)')
    ylabel(torquelabels{idx})
    legend('Left Aircraft','Right Aircraft')
end

torquelabels = {'TxTotal (N-m)','TyTotal (N-m)','TzTotal (N-m)'};
for idx = 1:3
    figure()
    hold on
    for idn = 1:nCrafts
        plot(time,torque_jointB(idx,:,idn)+torque_aeroB(idx,:,idn),'k','linestyle',linS{idn},'LineWidth',3)
    end
    xlabel('Time (sec)')
    ylabel(torquelabels{idx})
    legend('Left Aircraft','Right Aircraft')
end
%%%Control Effort
controllabels = {'Aileron (deg)','Elevator (deg)'};
for idx = 1:2
    figure()
    hold on
    for idn = 1:nCrafts
        plot(time,180/pi*dae_vec(idx,:,idn),'k','linestyle',linS{idn},'LineWidth',3)
    end
    xlabel('Time (sec)')
    ylabel(controllabels{idx})
    legend('Left Aircraft','Right Aircraft')
end
