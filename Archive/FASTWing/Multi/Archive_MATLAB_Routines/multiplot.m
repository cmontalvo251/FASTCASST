%%%Plotting Multiple runs on top of each other
purge
%function plotstates(arg1)
global NSTATES DELTA NOISE DUAL mass I_b I_b_inv num_ac rcg2ac

NSTATES = 15;
STATES = 1;
DELTA = 0;
CONTROLS = 0;
FORCES = 0;
BLEND = 0;
MOVIE = 0;
WINDS = 0;
RUNCODE = 0;

if exist('arg1','var')
    states = arg1;
else
    states = 1:15;
end

states = [3 4 5 6];

W2W = 0;
LATTICE = 1;

if W2W
    filenames = {'Meta1_1','Meta2_2','Meta3_3','Meta4_4','Meta5_5'};
    LegendNames = {'1 Aircraft','2 W2W','3 W2W','4 W2W','5 W2W'};
else
    filenames = {'Meta1_1','Meta1_2','Meta1_3','Meta1_4','Meta1_5'};
    LegendNames = {'1 Aircraft','2 T2T','3 T2T','4 T2T','5 T2T'};
end

if LATTICE
    filenames = {'Meta1_1','Meta2_4','Meta3_9'};
    LegendNames = {'1 Aircraft','2x2 Lattice','3x3 Lattice'};
end

h = [];
f = [];


for ll = 1:length(filenames)
    %%%PLOTTING ROUTINE
    filenames{ll}
    data = dlmread(['source/Out_Files/Controls_Paper/',filenames{ll}]);
    xout = data(:,2:end);
    xout = xout';
    tout = data(:,1);
    dim1 = 'm';
    [c,r] = size(xout);
    xSingle = 0.*xout;
    num_ac = round(c/NSTATES)
    
    %%%%%Convert state to single state by simply averaging out the data
    for jj = 1:num_ac
        for ii = 1:12
            xSingle(ii,:) = xSingle(ii,:) + (1/num_ac)*xout((jj-1)*NSTATES+ii,:);
        end
    end    
    num_ac = 1;
    
    %%%%Reduce the number of points
    if ll < 4
        skip = 1;
    else
        skip = 10;
    end
    xSingle = xSingle(:,1:skip:end);
    tout = tout(1:skip:end);

    Names = {'X_M','Y_M','Z_M','Phi','Theta','Psi','U','V','W','P','Q','R','zint','xint','yint'};
    ylabels = {['x_M(',dim1,')'],['y_M(',dim1,')'],['z_M(',dim1,')'],'\phi(deg)','\theta(deg)','\psi_M(deg)',['u(',dim1,'/s)'],['v(',dim1,'/s)'],['w(',dim1,'/s)'],'p(rad/s)','q(rad/s)','r(rad/s)','zint','xint','yint'};
    units = 1;
    LineWidth = 2;
    %colors = {'b','r','g','m','c','k','y'};
    colors = {'k','k'};
    linetype = {'-','--','-.','-s','-x','-.'};
    for ii = states
      if ii <= 15
        if ii >= 4 && ii <= 6
          factor = 180/pi;
        else
          factor = units;
        end
        if ii >= 10
          factor = 1;       
        end
        if ll == 1
            [h(ii),f(ii)] = plottool(1,Names{ii},12,'Time(sec)',ylabels{ii});
            %xlim([0 50])
        end
        p(1) = plot(f(ii),tout,factor.*xSingle(ii,:),[colors{1},linetype{ll}],'LineWidth',LineWidth);
        for jj = 2:num_ac
          val = (jj-1)*NSTATES+ii;
          jdx = jj;
          while jdx > 7
              jdx = jdx - 7;
          end
          p(jj) = plot(tout,factor.*xout(val,:),[colors{jdx},linetype{2}],'LineWidth',LineWidth);
        end
      end
    end
end
for ii = states
if exist('LegendNames','var')
  legend(f(ii),LegendNames)
end
end




