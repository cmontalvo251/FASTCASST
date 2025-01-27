function xstate = SixDOF(xinitial1,time,controller,nCrafts)
global da de Fymag dely

apprentice_properties
% foamFlyer_properties

dt = time(2) - time(1);

xstate(:,1) = xinitial1;
xinitial2 = xinitial1 + [0 b 0 0*pi/180 0*pi/180 0 0 0 0 0 0 0]';

if nCrafts == 1
    for k = 1:length(time)-1
        %%%Runge_Kutta
        xk = xstate(:,k);
        tk = time(k);
        disp(tk)
        k1 = Derivatives(xk,tk,controller);
    %     de_vec(k+1,1) = de;  %Plot control
    %     da_vec(k+1,1) = da;
        k2 = Derivatives(xk+k1*dt/2,tk+dt/2,controller);
        k3 = Derivatives(xk+k2*dt/2,tk+dt/2,controller);
        k4 = Derivatives(xk+k3*dt,tk+dt,controller);
        phi = (1/6)*(k1 + 2*k2 + 2*k3 + k4);
        xstate(:,k+1) = xstate(:,k) + phi*dt;
    end
end
%------------------------------------------------------------------------%
if nCrafts > 1
%     Fymag_vec = zeros(length(time),2);
    metaState(:,:,1) = [xinitial1, xinitial2];
    for k = 1:length(time)-1
            xk = metaState(:,:,k);
            tk = time(k);
            phi = zeros(12,2);
        for n = 1:nCrafts
            %%%Runge_Kutta
            [k1(1:12,:)] = metaDerive(xk,tk,controller,n);
            %     de_vec(k+1,1) = de;  %Plot control
            %     da_vec(k+1,1) = da;
%             dely_vec(k+1,n) = dely;
%             Fymag_vec(k+1,n) = Fymag;
            k2(1:12,:) = metaDerive(xk+k1*dt/2,tk+dt/2,controller,n);
            k3(1:12,:) = metaDerive(xk+k2*dt/2,tk+dt/2,controller,n);
            k4(1:12,:) = metaDerive(xk+k3*dt,tk+dt,controller,n);
            phi = phi + (1/6)*(k1 + 2*k2 + 2*k3 + k4);
        end
        metaState(:,:,k+1) = metaState(:,:,k) + phi*dt;
    end
end
%------------------------------------------------------------------------%

plotting = 1;
if plotting == 1
    if nCrafts > 1
        ylabels = {'x','y','z','\phi','\theta','\psi','u','v','w','p','q','r'}';
        for idx = 1:6
            figure();
            linS = {'-','--'};
            for idn = 1:nCrafts
                state(idx,:) = metaState(idx,idn,:);
                plot(time,state(idx,:),'linestyle',linS{idn})
                xlabel('Time (sec)')
                ylabel(ylabels{idx})
                hold on
            end
            legend('Left','Right')
            hold off
        end
        % figure
        % plot(dely_vec(:,1),Fcy_vec(:,1)))
        % xlabel('Delta y');ylabel('Force');
        % axis([-.1 .1 -5 5])

        % figure
        % plot(time,dely_vec(:,1),'b-',time,dely_vec(:,2),'r--')
        % ylabel('Delta Y');legend('left','right')
        % figure
        % plot(time,Fymag_vec(:,1),'b-',time,Fymag_vec(:,2),'r--')
        % ylabel('Fmagnetic_y');legend('left','right')
        % figure
        % plot(time,de_vec);xlabel('time');ylabel('de')
        % figure
        % plot(time,da_vec);xlabel('time');ylabel('da')
    end
    if nCrafts == 1
        ylabels = {'x','y','z','\phi','\theta','\psi','u','v','w','p','q','r'}';
        for idx = 1:6
            figure();
            plot(time,xstate(idx,:))
            xlabel('Time (sec)')
            ylabel(ylabels{idx})
        end
    end
end
%     figure
%     subplot(3,1,1)
%     plot(time,xstate(1,:));xlabel('Time(s)');ylabel('x')
%     subplot(3,1,2)
%     plot(time,xstate(2,:));ylabel('y')
%     subplot(3,1,3)
%     plot(time,-xstate(3,:));ylabel('z')
% 
%     figure
%     subplot(3,1,1)
%     plot(time,xstate(4,:));xlabel('Time(s)');ylabel('\phi')
%     subplot(3,1,2)
%     plot(time,xstate(5,:));ylabel('\theta')
%     subplot(3,1,3)
%     plot(time,-xstate(6,:));ylabel('\psi')
% 
%     figure
%     subplot(3,1,1)
%     plot(time,xstate(7,:));xlabel('Time(s)');ylabel('u')
%     subplot(3,1,2)
%     plot(time,xstate(8,:));ylabel('v')
%     subplot(3,1,3)
%     plot(time,-xstate(9,:));ylabel('w')

%     figure
%     subplot(3,1,1)
%     plot(time,xstate(10,:));xlabel('Time(s)');ylabel('p')
%     subplot(3,1,2)
%     plot(time,xstate(11,:));ylabel('q')
%     subplot(3,1,3)
%     plot(time,-xstate(12,:));ylabel('r')

    % figure
    % plot(time,xstate(6,:)*180/pi);ylabel('\psi (degrees)');

    % figure 
    % plot(time,da_vec*180/pi);ylabel('da (degrees)');
    % 
    % figure
    % plot(fly_time1,fly_phi1,'*');
    % title('Roll test1 maneuver 1');xlabel('time(s)');ylabel('\phi (degrees)');
    % hold on
    % plot(time,xstate(4,:)*180/pi,'r');legend('actual','simulated');

    % figure
    % plot(fly_time2,fly_phi2,'*');
    % title('Roll test 1 maneuver 2');xlabel('time(s)');ylabel('\phi (degrees)');
    % hold on
    % plot(time,xstate(4,:)*180/pi,'r');legend('actual','simulated');
