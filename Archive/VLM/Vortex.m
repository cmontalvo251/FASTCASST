purge


%%%%%%%%VORTEX LATTICE METHOD CODE%%%%%%%%%%

PLOTWING = 0;
PLOTMESH = 0;
PLOTPOINTS = 0;
PLOTTOTALS = 1;
PLOTHORSHOE = 0;

%%%%%%%MOTHERSHIP WING PROPERTIES%%%%%%%%

spans = [10:-0.1:2.04,2.04];
chords = 0;%[-10:1:10];
%Clavec = zeros(20,20);
%Claequivvec = zeros(20,20);
mmdx = 0;
nndx = 0;
%plottool(1,'Wing',12,'chord','span');
for mm = 2
  nndx = 0;
  mmdx = mmdx + 1;
  nspanpanels = mm
  cla
  %for nchordpanels = 1
  for nn = spans
    nn
    nndx = nndx+1;
    lldx = 0;
    for ll = chords
      lldx = lldx + 1;

chord = 0.3;
wingspan = 2.04;
nchordpanels = 1;
%nspanpanels = 100;


%%%%%%%%CHILD WING LOCATION%%%%%%%%%%%%%%%
%%%ASSUME THAT WING PROPERTIES ARE THE SAME%%

NAC = 2;
xlocation = ll; %%using origin of mothership frame
ylocation = nn;

%%%%%%%%%%%%%%STEP 1 Break Wing Into Panel Sections%%%%%%%%%%

Sarea = wingspan*chord;
nspan = nspanpanels;
nchord = nchordpanels;
ntotal = nspan*nchord;
dxpanel = chord/nchord;
dypanel = (wingspan)/nspan;
l = dxpanel;

%%%MOTHERSHIP
CONTROLPOINTS = zeros(nspan*nchord,2);
counter = 0;
for ii = 1:nchord
  chordloc = (ii-1)*dxpanel + 3*l/4;
  for jj = 1:nspan
    counter = counter+1;
    spanloc = (jj-1)*dypanel + dypanel/2;
    CONTROLPOINTS(counter,:) = [chordloc,spanloc];
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
VORTEXA = 0.*CONTROLPOINTS;
VORTEXB = 0.*CONTROLPOINTS;
counter = 0;
for ii = 1:nchord
  chordloc = (ii-1)*dxpanel + l/4;
  for jj = 1:nspan
    counter = counter + 1;
    spana = (jj-1)*dypanel;
    spanb = (jj)*dypanel;
    VORTEXA(counter,:) = [chordloc,spana];
    VORTEXB(counter,:) = [chordloc,spanb];
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

%%%%%%%%%%%%%%%%%%%PLOT WING,CONTROL POINTS AND VORTICES%%%%%

if PLOTWING
  %plottool(1,'Wing',12,'chord','span');
  rectangle('Position',[0 0 chord wingspan])
  rectangle('Position',[xlocation ylocation chord wingspan])
  plot(CONTROLPOINTS(:,1),CONTROLPOINTS(:,2),'b*')
  plot(CONTROLPOINTSCHILD(:,1),CONTROLPOINTSCHILD(:,2),'b*')
  for ii = 1:ntotal
    plot([VORTEXA(ii,1) VORTEXB(ii,1)],[VORTEXA(ii,2) VORTEXB(ii,2)],'r-','LineWidth',2)
    plot([VORTEXA(ii,1) chord*2],[VORTEXA(ii,2) VORTEXA(ii,2)],'k-','LineWidth',2)
    plot([VORTEXB(ii,1) chord*2],[VORTEXB(ii,2) VORTEXB(ii,2)],'k-','LineWidth',2)
    plot([VORTEXACHILD(ii,1) VORTEXBCHILD(ii,1)],[VORTEXACHILD(ii,2) VORTEXBCHILD(ii,2)],'r-','LineWidth',2)
    plot([VORTEXACHILD(ii,1) xlocation+chord*2],[VORTEXACHILD(ii,2) VORTEXACHILD(ii,2)],'k-','LineWidth',2)
    plot([VORTEXBCHILD(ii,1) xlocation+chord*2],[VORTEXBCHILD(ii,2) VORTEXBCHILD(ii,2)],'k-','LineWidth',2)
  end
  axis equal
  ax = axis;
end

%%%%%%%%%%%%STEP 2,3 FIND THE DOWNWASH AT ALL CONTROL POINTS DUE TO
%%%%%%%%%%%%%EACH HORSHOE VORTEX%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%CONCATENATE CONTROLPOINTS AND VORTICES%%%%%%%%%%%%%%%%%
CONTROLPOINTS = [CONTROLPOINTS;CONTROLPOINTSCHILD];
VORTEXA = [VORTEXA;VORTEXACHILD];
VORTEXB = [VORTEXB;VORTEXBCHILD];

ntotal = ntotal*NAC;
C = zeros(ntotal,ntotal);
if PLOTHORSHOE
  figure()
end
for ii = 1:ntotal
  %%ii = control point
  %%CONTROLPOINTS(ii,:) -- x,y coordinate for control point
  for jj = 1:ntotal
    %%jj = horshoe vortex
    %%VORTEXA(jj,:) -- x,y coordinate for A point on vortex
    %%VORTEXB(jj,:) -- x,y coordinate for B point on vortex
    w = Horshoe(CONTROLPOINTS(ii,:),VORTEXA(jj,:),VORTEXB(jj,:));
    C(ii,jj) = w;
    if PLOTHORSHOE
      plot(CONTROLPOINTS(ii,1),CONTROLPOINTS(ii,2),'b*')
      hold on
      plot(VORTEXA(jj,1),VORTEXA(jj,2),'r*')
      plot(VORTEXB(jj,1),VORTEXB(jj,2),'r*')
      axis(ax)
      drawnow
      cla
    end
  end
end

%%%%%%%%STEP 4 SOLVE SYSTEM OF EQUATIONS%%%%%%%%%%%%%%%%
%%%%WE NOW HAVE A SYSTEM OF EQUATIONS W=C*G%%%%%%%%%%%%%

w = -ones(ntotal,1);
g = inv(C)*w;

%%%%%%%%%%%%EXTRACT MOTHER AND CHILD%%%%%%%%%%%%%%%

gmothership = g(1:ntotal/2);
gchild = g(ntotal/2+1:end);
CONTROLPOINTSM = CONTROLPOINTS(1:ntotal/2,:);
CONTROLPOINTSC = CONTROLPOINTS(ntotal/2+1:end,:);

%%%%%%%%%%LETS PLOT G FOR KICKS%%%%%%%%%%%%%%%%%%%%%%

if PLOTMESH
  minX = min(CONTROLPOINTSM(:,1));
  maxX = max(CONTROLPOINTSM(:,1));
  X = minX:dxpanel:maxX;
  minY = min(CONTROLPOINTSM(:,2));
  maxY = max(CONTROLPOINTSM(:,2));
  Y = minY:dypanel:maxY;
  [xx,yy] = meshgrid(X,Y);
  zz = xx;
  counter = 0;
  for ii = 1:nchord
    for jj = 1:nspan
      counter = counter + 1;
      xloc = find(abs(xx(jj,ii)-CONTROLPOINTSM(:,1))<dxpanel/2);
      yloc = find(abs(yy(jj,ii)-CONTROLPOINTSM(:,2))<dypanel/2);
      loc = sort([xloc;yloc]);
      l = find(loc(2:end)-loc(1:end-1)==0);
      zz(jj,ii) = -gmothership(loc(l))+0.1;
    end
  end
  mesh(xx,yy,zz)
  minX = min(CONTROLPOINTSC(:,1));
  maxX = max(CONTROLPOINTSC(:,1));
  X = minX:dxpanel:maxX;
  minY = min(CONTROLPOINTSC(:,2));
  maxY = max(CONTROLPOINTSC(:,2));
  Y = minY:dypanel:maxY;
  [xx,yy] = meshgrid(X,Y);
  zz = xx;
  counter = 0;
  for ii = 1:nchord
    for jj = 1:nspan
      counter = counter + 1;
      xloc = find(abs(xx(jj,ii)-CONTROLPOINTSC(:,1))<dxpanel/2);
      yloc = find(abs(yy(jj,ii)-CONTROLPOINTSC(:,2))<dypanel/2);
      loc = sort([xloc;yloc]);
      l = find(loc(2:end)-loc(1:end-1)==0);
      zz(jj,ii) = -gchild(loc(l))+0.1;
    end
  end
  mesh(xx,yy,zz)
end
if PLOTPOINTS
  for ii = 1:ntotal
    plot3(CONTROLPOINTS(ii,1),CONTROLPOINTS(ii,2),-g(ii),'r*')
  end
end
drawnow
view(90,0)
z = '0000';
num = num2str(nndx);
Name = ['Lift_',z(1:4-length(num)),num];
%saveas(gcf,Name,'png');
cla
%%%%%%%%%%%%%%%%%%%%%%STEP 5 %%%%%%%%%%%%%%%%%%%%%
%%%%%%NOW WE CAN COMPUTE LIFT AND DRAG%%%%%%%%%%%%
Clam = 0;
Clac = 0;
for ii = 1:ntotal/2
  Clam = Clam - gmothership(ii)*dypanel;
  Clac = Clac - gchild(ii)*dypanel;
end
Claequivvecm(lldx,nndx) = 2*Clam/Sarea;
Claequivm = 2*Clam/Sarea;
Claequivc = 2*Clac/Sarea;
Claequivvecc(lldx,nndx) = 2*Clac/Sarea;
    
    end
  end
end
if PLOTTOTALS
  Claequivvecm(1)
  plot(spans,Claequivvecm./Claequivvecm(1))
  hold on
  load Lift.mat
  plot(spans,Cla100./Cla100(1),'g-')
end

% ClaMeshP = Claequivvecm;
% mesh(ClaMeshP)
% ClaMeshC = Claequivvecc;
% figure()
% mesh(ClaMeshC)
