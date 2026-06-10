close all
%clear

% stable = 'Unstable';
% posmodes = [0.18659+4.9103i;0.037923+0.05027i];
% dom_modes = ['-x1,-x2,-x3,-x4,-y1,-y2,-y3,-y4';'-p1,-x2,-x3,-x4,-y1,-y2,-y3,-q4'];

[h1,f1] = plottool(1,'Aircraft_Configuration',12);
view(-90,90);
font = 10;

%U = [0 0 1; 0 1 1; 1 1 0];

AIRPLANE = 0;

if AIRPLANE
  if ~exist('STLREAD','var')
    STLREAD = 0;
  end
  if isempty(STLREAD) || STLREAD == 0
    [xf,yf,zf]=stlread('Fuselage.STL');
    [xL,yL,zL]=stlread('LWing.STL');
    [xR,yR,zR]=stlread('RWing.STL');
    [xp,yp,zp]=stlread('Prop.STL');
    xyzf = [xf;yf;zf];
    xyzL = [xL;yL;zL];
    xyzR = [xR;yR;zR];
    xyzp = [xp;yp;zp];
    STLREAD = 1;
  end
  wingspan = 2.04;
else
  wingspan = 0.4;
end
Uii = U(:,:,ii);
Uii = Uii(end:-1:1,:);
Uii = Uii(:,end:-1:1);
maxs = 3;
for s = 1:3
  for e = 1:3
    if Uii(s,e) == 1
      if s < maxs
	maxs = s;
      end
      x = wingspan*(s-1);
      y = wingspan*(e-1);
      z = 0;
      state = [x;y;z;0;0;0];
      if AIRPLANE
	draw_plane_fancy(state,xyzf,xyzL,xyzR,xyzp,wingspan,0);
      else
	text(state(1),state(2),'1','FontSize',font);
      end
      %draw_plane_fancy(state,xyzf,xyzL,xyzR,xyzp,wingspan,0,0);
    else
      if ~AIRPLANE
	x = wingspan*(s-1);
	y = wingspan*(e-1);
	z = 0;
	state = [x;y;z;0;0;0];
	text(state(1),state(2),'0','FontSize',font);
      end
    end
  end
end
axis([-2 5 -2 6])
limits = axis;
minx = min([limits(1),limits(2)]);
meany = 0.5*(limits(3)+limits(4));
%if strcmp(stable,'Stable')
  %text(minx,meany+0.5,stable,'FontSize',16)
%elseif strcmp(stable,'Unstable')
  %text(maxs-1.2-(3-maxs)*1,meany+0.65,stable,'FontSize',16)
  ctr = 1;
  for s = 1:length(posmodes)
    if imag(posmodes(s)) >= 0
      rmode = posmodes(s);
      if AIRPLANE
	xpos = maxs-1.2-(3-maxs)*1-0.5*(ctr);
	ypos1 = meany+4;
	ypos2 = meany+1.3;
      else
	maxs = 0.5;
	xpos = 3.7-(3-maxs)*1-0.4*(ctr);
	shift = -6.5;
	ypos1 = meany+4+shift;
	ypos2 = meany+1.3+shift;
      end
      text(xpos,ypos1,num2str(posmodes(s),3),'FontSize',font)
      text(xpos,ypos2,dominant_modes{s},'FontSize',font)
      ctr = ctr + 1;
    end
  end
%end
axis([-4 5 -12 0])
axis off 
grid off
%set(h1,'position',[308   169   872   340])



