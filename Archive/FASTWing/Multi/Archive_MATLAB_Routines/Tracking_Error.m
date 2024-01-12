purge

global NSTATES num_ac mass I_b I_b_inv rcg2ac phic thetac

NSTATES = 15;

tic

system('rm Run.exe');
system('rm Tracking_Performance.txt');
system('make rebuild');

maxrows = 3;
maxcols = 3;

config_data = {'20 20 !Grid_Size_of_Configuration'};
for ii = 1:20
    config_data = [config_data;num2str(zeros(1,10))];
end
config_data = [config_data;'10000 10000 !KLtangent_KLnormal';'40 40 !CLtangent_CLnormal';'1000 !Maximum_Magnet_Force';'372.1041 2584.05625 2584.05625 !KR';'1.4884 10.33623 10.33623 !CR'];

for rows = 1:maxrows
    for cols = 1:maxcols
        
        num_ac = rows*cols;
        
        if num_ac <= 40
            
                %%%%Write Configuration file
                writematrix = zeros(20,20);
                configuration = ones(rows,cols)
                writematrix(1:rows,1:cols) = configuration;
            
                for ii = 1:20
                    config_data{ii+1} = num2str(writematrix(ii,:));
                end
            
                %%%writedata to file
                writedata('source/Input_Files/Configuration.CONN',config_data);

                %%%COMPUTE SPEED FIRST
                %%%Compute Speed of given configuration
                %%First comptute maximum pitch rate
                disp('Compute Pitch Rate Dynamics')
                phic = 0;
                thetac = 30*pi/180;
                [wnq,kpz,d,d,qmax,d] = ComputeSpeed();
                %%Then compute maximum roll rate
                disp('Compute Roll Rate Dynamics')
                thetac = 0;
                phic = 30*pi/180;
                [d,d,wnp,kppsi,d,pmax] = ComputeSpeed();

                %%%Write all this data to a file
                fid = fopen('source/Input_Files/Meta.FREQ','w');
                data = {[num2str(wnq),' !wnq'];
                    [num2str(kpz),' !kpz'];
                    [num2str(wnp),' !wnp'];
                    [num2str(kppsi),' !kppsi'];
                    [num2str(qmax),' !qmax'];
                    [num2str(pmax),' !pmax']};
                for ii = 1:length(data)
                  fprintf(fid,'%s \n',data{ii});
                end
            
            for yyt = 1:100 %%Run it 100 times
                
                yyt
            

                %%%Run code
                system('Run.exe source/MultiMeta.ifiles 1> runfile');   
                %%Read Data and get nominal tracking performance
                data = dlmread('source/Out_Files/Meta.OUT');
                tout = data(:,1);
                xout = data(:,2:end);

                [r,c] = size(xout);
                xSingle = zeros(r,12);
                num_ac = round(c/NSTATES);

                %%%%%Convert state to single state by simply averaging out the data
                for jj = 1:num_ac
                    for ii = 1:12
                        xSingle(:,ii) = xSingle(:,ii) + (1/num_ac)*xout(:,(jj-1)*NSTATES+ii);
                    end
                end    
                num_ac = 1;

                %%%Compute perterbations from trim
                xSingle(:,1) = xSingle(:,1);
                xSingle(:,2) = xSingle(:,2)-cols*1.02+1.02;
                xSingle(:,3) = xSingle(:,3) + 200;
                %%theta
                xSingle(:,5) = xSingle(:,5) - mean(xSingle(:,5));
                %%%psi
                xSingle(:,6) = xSingle(:,6) - mean(xSingle(:,6));
                %%%Process data
                meanX = mean(abs(xSingle));
                minX = min(abs(xSingle));
                maxX = max(abs(xSingle));
                stdX = std(xSingle);
                %%%Append data to file
                dlmwrite('Tracking_Performance.txt',[num2str(rows),' ',num2str(cols)],'delimiter',' ','-append');
                dlmwrite('Tracking_Performance.txt',[meanX;minX;maxX;stdX],'delimiter',' ','-append');
            end
        end
    end
end


toc