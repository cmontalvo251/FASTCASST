purge

system('rm Run.exe');
system('rm Magnet.txt');
system('~/Dropbox/BlackBox/compilec forcesc.cpp -O1 -w');

%%%Find limits for zdot = ?
zdot = 15;

plottool(1,'MagnetTravel',12,'x','y','z','',[-27,30]);

%%Create initial condition file
step = 0.001;
xbreak = 0;
ii = 0;
contact = 1;
while contact == 1
    ii = ii + 1;
    contact = 0;
    x = ii*step;y = 0;z = 0;xdot = 0;ydot = 0;
    xinitial = [x;y;z;xdot;ydot;zdot]
    dlmwrite('Initial.txt',[x;y;z;xdot;ydot;zdot],'delimiter',' ');
    %%Run code
    system('./Run.exe');
    %%%Extract results
    data = dlmread('Magnet.txt');
    %%%Plot it
    t = data(:,1);
    xyz = data(:,2:end);
    cla;
    plot3(xyz(:,1),xyz(:,2),xyz(:,3),'b-','LineWidth',2)
    CubeDraw(0.0508,0.0508,0.0213,0,0,1,0,0,0,'k')
    axis equal
    drawnow
    %%Check contact
    if t(end) < 10
        if (abs(xyz(end,1)) < 0.0508/2)
            if (abs(xyz(end,2)) < 0.0508/2)
                contact = 1
                xbreak = x;
                tbreak = t(end)
            end
        end
    end
end

%%Now check to see if xbreak is bigger or smaller than distance traveled of
%%wing
Vwing = 5
xwing = Vwing*tbreak
xbreak
tbreak

if xbreak > xwing
    disp('Velocity Possible')
else
    disp('Not Possible')
end


