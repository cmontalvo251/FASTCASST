purge

COMPILECODE = 1;
RUNCODE = 1;

if COMPILECODE
  system('rm Run.exe');
  system('make rebuild');
end

maxrows = 4;
maxcols = 4;

meanLDNom = zeros(maxrows,maxcols);
alfaLDmax = zeros(maxrows,maxcols);
Pincmax = zeros(maxrows,maxcols);
meanLDmax = zeros(maxrows,maxcols);
minLDNom = zeros(maxrows,maxcols);
maxLDNom = zeros(maxrows,maxcols);

config_data = {'20 20 !Grid_Size_of_Configuration'};
for ii = 1:20
    config_data = [config_data;num2str(zeros(1,10))];
end
config_data = [config_data;'10000 10000 !KLtangent_KLnormal';'40 40 !CLtangent_CLnormal';'1000 !Maximum_Magnet_Force';'372.1041 2584.05625 2584.05625 !KR';'1.4884 10.33623 10.33623 !CR'];

plottool(1,'LD',18,'AOA(deg)','L/D');
counter = 0;
linestyle = {'b-','r-','g-','m-','y-','k-','c-'};
LDavg_max = zeros(maxrows,maxcols);
PincNom = zeros(maxrows,maxcols);

for rows = 1:maxrows
    for cols = 1:maxcols
        
        %%%%Write Configuration file
        writematrix = zeros(20,20);
        configuration = ones(rows,cols)
        writematrix(1:rows,1:cols) = configuration;
           
        for ii = 1:20
            config_data{ii+1} = num2str(writematrix(ii,:));
        end
            
        %%%writedata to file
        writedata('source/Input_Files/Configuration.CONN',config_data);
        
        system('./Run.exe source/MultiMeta.ifiles 1>runfile');                    

        %%%Open Lift to Drag
        LD = dlmread('LifttoDrag.txt');
        L = LD(:,1:2:(2*rows*cols-1));
        D = LD(:,2:2:(2*rows*cols));
        LD = L./D;
        alfa = linspace(0,10,length(LD));
        counter = counter + 1;
	if counter > length(counter)
	  counter = 1;
	end
	if rows*cols > 1
	  LDavg = sum(LD')'/(rows*cols);
	else
	  LDavg = LD;
	end
        plot(alfa,LDavg,linestyle{counter},'LineWidth',2)
	LDavg_max(rows,cols) = max(LDavg);
	PincNom(rows,cols) = 100*(rows*cols)*(LDavg_max(rows,cols)/LDavg_max(1,1)-1);
    end
end

legend('Single Aircraft','4 Aircraft Wing Tip to Wing Tip')
plottool(1,'meanLDNom',12,'Number of Columns','Number of Rows');
[C,h] = contour(LDavg_max,'k','LineWidth',1);
clabel(C,h,'FontSize',10)

plottool(1,'PincNom',12,'Number of Columns','Number of Rows');
[C,h] = contour(PincNom,'k','LineWidth',1);
clabel(C,h,'FontSize',10)

break

plottool(1,'minLDNom',12,'Number of Columns','Number of Rows');
[C,h] = contour(minLDNom,'k','LineWidth',1);
clabel(C,h,'FontSize',10)

plottool(1,'maxLDNom',12,'Number of Columns','Number of Rows');
[C,h] = contour(maxLDNom,'k','LineWidth',1);
clabel(C,h,'FontSize',10)

plottool(1,'meanLDmax',12,'Number of Columns','Number of Rows');
[C,h] = contour(meanLDmax,'k','LineWidth',1);
clabel(C,h,'FontSize',10)

plottool(1,'PincMax',12,'Number of Columns','Number of Rows');
[C,h] = contour(Pincmax,'k','LineWidth',1);
clabel(C,h,'FontSize',10)

plottool(1,'Alfa',12,'Number of Columns','Number of Rows');
[C,h] = contour(alfaLDmax,'k','LineWidth',1);
clabel(C,h,'FontSize',10)

