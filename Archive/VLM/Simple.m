purge

tic

%%%%%%COMPUTE WHEN VERY FAR AWAY

NAC = 2;
chord = 0.3215;
wingspan = 2.04;
nchordpanels = 1;
nspanpanels = 2; %This effects L0 when far away

%%%%%%%%CHILD WING LOCATION%%%%%%%%%%%%%%%
%%%ASSUME THAT WING PROPERTIES ARE THE SAME%%

xlocation = 10000;
ylocation = 10000;

%%%%%%%%%%%%%%STEP 1 Break Wing Into Panel Sections%%%%%%%%%%

Sarea = wingspan*chord;
nspan = nspanpanels;
nchord = nchordpanels;
ntotal = nspan*nchord;
dxpanel = chord/nchord;
dypanel = (wingspan)/nspan;
l = dxpanel;

%%%MOTHERSHIP
CONTROLPOINTSPARENT = zeros(nspan*nchord,2);
counter = 0;
for ii = 1:nchord
  chordloc = (ii-1)*dxpanel + 3*l/4;
  for jj = 1:nspan
    counter = counter+1;
    spanloc = (jj-1)*dypanel + dypanel/2;
    CONTROLPOINTSPARENT(counter,:) = [chordloc,spanloc];
  end
end

%%CHILD
CONTROLPOINTSCHILD = zeros(nspan*nchord,2);
counter = 0;
for ii = 1:nchord
  chordloc = (ii-1)*dxpanel + 3*l/4+xlocation;
  for jj = 1:nspan
    counter = counter+1;
    spanloc = (jj-1)*dypanel + dypanel/2+ylocation;
    CONTROLPOINTSCHILD(counter,:) = [chordloc,spanloc];
  end
end

%%%%%%%%%%%%MOTHERSHIP%%%%%%%%%%%%%%%%%%%%%%%%%%
VORTEXAPARENT = 0.*CONTROLPOINTSPARENT;
VORTEXBPARENT = 0.*CONTROLPOINTSPARENT;
counter = 0;
for ii = 1:nchord
  chordloc = (ii-1)*dxpanel + l/4;
  for jj = 1:nspan
    counter = counter + 1;
    spana = (jj-1)*dypanel;
    spanb = (jj)*dypanel;
    VORTEXAPARENT(counter,:) = [chordloc,spana];
    VORTEXBPARENT(counter,:) = [chordloc,spanb];
  end
end
%%%%%%%%%%%%%%CHILD%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
VORTEXACHILD = 0.*CONTROLPOINTSCHILD;
VORTEXBCHILD = 0.*CONTROLPOINTSCHILD;
counter = 0;
for ii = 1:nchord
  chordloc = (ii-1)*dxpanel + l/4+xlocation;
  for jj = 1:nspan
    counter = counter + 1;
    spana = (jj-1)*dypanel+ylocation;
    spanb = (jj)*dypanel+ylocation;
    VORTEXACHILD(counter,:) = [chordloc,spana];
    VORTEXBCHILD(counter,:) = [chordloc,spanb];
  end
end

%%%%%%%%%%%%STEP 2,3 FIND THE DOWNWASH AT ALL CONTROL POINTS DUE TO
%%%%%%%%%%%%%EACH HORSHOE VORTEX%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%CONCATENATE CONTROLPOINTS AND VORTICES%%%%%%%%%%%%%%%%%
CONTROLPOINTS = [CONTROLPOINTSPARENT;CONTROLPOINTSCHILD];
VORTEXA = [VORTEXAPARENT;VORTEXACHILD];
VORTEXB = [VORTEXBPARENT;VORTEXBCHILD];

ntotal = ntotal*NAC;
C = zeros(ntotal,ntotal);
for ii = 1:ntotal
  %%ii = control point
  %%CONTROLPOINTS(ii,:) -- x,y coordinate for control point
  for jj = 1:ntotal
    %%jj = horshoe vortex
    %%VORTEXA(jj,:) -- x,y coordinate for A point on vortex
    %%VORTEXB(jj,:) -- x,y coordinate for B point on vortex
    w = Horshoe(CONTROLPOINTS(ii,:),VORTEXA(jj,:),VORTEXB(jj,:));
    C(ii,jj) = w;
  end
end

%%%%%%%%STEP 4 SOLVE SYSTEM OF EQUATIONS%%%%%%%%%%%%%%%%
%%%%WE NOW HAVE A SYSTEM OF EQUATIONS W=C*G%%%%%%%%%%%%%

w = -ones(ntotal,1);
g = inv(C)*w;

%%%%%%%%%%%%EXTRACT MOTHER AND CHILD%%%%%%%%%%%%%%%

gmothership = g(1:ntotal/2);
gchild = g(ntotal/2+1:end);

break

%%%%%%%%%%%%%%%%%%%%%%STEP 5 %%%%%%%%%%%%%%%%%%%%%
%%%%%%NOW WE CAN COMPUTE LIFT AND DRAG%%%%%%%%%%%%
Clam = 0;
Clac = 0;
for ii = 1:ntotal/2
  Clam = Clam - 2*gmothership(ii)*(dypanel)/Sarea;
  Clac = Clac - 2*gchild(ii)*(dypanel)/Sarea;
end
Clam0 = Clam
Clac0 = Clac

%%%%%%%%%%%%%NOW DO IT AGAIN FOR ACTUAL VALUE%%%%%%%%%

xcgparent = 0;
ycgparent = 0;

xrootparent = chord/2;
yrootparent = 0;

xcgchild = 0;
ycgchild = 2.04;

xrootchild = chord/2;
yrootchild = -wingspan;

xlocation = -(xcgchild + xrootchild - xcgparent - xrootparent)
ylocation = ycgchild + yrootchild - ycgparent - yrootparent

%%%%%%%%%%%%%%STEP 1 Break Wing Into Panel Sections%%%%%%%%%%

%%CHILD
counter = 0;
for ii = 1:nchord
  chordloc = (ii-1)*dxpanel + 3*l/4+xlocation;
  for jj = 1:nspan
    counter = counter+1;
    spanloc = (jj-1)*dypanel + dypanel/2+ylocation;
    CONTROLPOINTSCHILD(counter,:) = [chordloc,spanloc];
  end
end

%%%%%%%%%%%%%%CHILD%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
VORTEXACHILD = 0.*CONTROLPOINTSCHILD;
VORTEXBCHILD = 0.*CONTROLPOINTSCHILD;
counter = 0;
for ii = 1:nchord
  chordloc = (ii-1)*dxpanel + l/4+xlocation;
  for jj = 1:nspan
    counter = counter + 1;
    spana = (jj-1)*dypanel+ylocation;
    spanb = (jj)*dypanel+ylocation;
    VORTEXACHILD(counter,:) = [chordloc,spana];
    VORTEXBCHILD(counter,:) = [chordloc,spanb];
  end
end

%%%%%%%%%%%%STEP 2,3 FIND THE DOWNWASH AT ALL CONTROL POINTS DUE TO
%%%%%%%%%%%%%EACH HORSHOE VORTEX%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%CONCATENATE CONTROLPOINTS AND VORTICES%%%%%%%%%%%%%%%%%
CONTROLPOINTS = [CONTROLPOINTSPARENT;CONTROLPOINTSCHILD];
VORTEXA = [VORTEXAPARENT;VORTEXACHILD];
VORTEXB = [VORTEXBPARENT;VORTEXBCHILD];

C = zeros(ntotal,ntotal);
for ii = 1:ntotal
  %%ii = control point
  %%CONTROLPOINTS(ii,:) -- x,y coordinate for control point
  for jj = 1:ntotal
    %%jj = horshoe vortex
    %%VORTEXA(jj,:) -- x,y coordinate for A point on vortex
    %%VORTEXB(jj,:) -- x,y coordinate for B point on vortex
    w = Horshoe(CONTROLPOINTS(ii,:),VORTEXA(jj,:),VORTEXB(jj,:));
    C(ii,jj) = w;
  end
end

%%%%%%%%STEP 4 SOLVE SYSTEM OF EQUATIONS%%%%%%%%%%%%%%%%
%%%%WE NOW HAVE A SYSTEM OF EQUATIONS W=C*G%%%%%%%%%%%%%

w = -ones(ntotal,1);
g = inv(C)*w;

%%%%%%%%%%%%EXTRACT MOTHER AND CHILD%%%%%%%%%%%%%%%

gmothership = g(1:ntotal/2);
gchild = g(ntotal/2+1:end);

%%%%%%%%%%%%%%%%%%%%%%STEP 5 %%%%%%%%%%%%%%%%%%%%%
%%%%%%NOW WE CAN COMPUTE LIFT AND DRAG%%%%%%%%%%%%
Clam = 0;
Clac = 0;
for ii = 1:ntotal/2
  Clam = Clam - 2*gmothership(ii)*(dypanel)/Sarea;
  Clac = Clac - 2*gchild(ii)*(dypanel)/Sarea;
end
Clam
Clac
f0 = Clam/Clam0
f1 = Clac/Clac0

l0 = (wingspan/2)*(1/(4*chord))*(Clam-Clam0)
l1 = (wingspan/2)*(1/(4*chord))*(Clac-Clac0)

%min(g) =~ =0.64