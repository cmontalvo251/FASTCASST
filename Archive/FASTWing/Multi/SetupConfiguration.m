%%Read in configuration
alldata = importdata('Input_Files/Configuration.CONN');
configuration = alldata.data(1:20,1:20);
num_ac = sum(sum(configuration));

%%Compute composite c.g and distance vectors from c.g to each aircraft
rcg2ac = zeros(2,num_ac);
xyzAC = zeros(2,num_ac);
idx = 0;
num_col = 1;
for ii = 1:20
    for jj = 1:20
        if configuration(ii,jj) == 1
	    if jj > num_col
	      num_col = jj;
	    end
            idx = idx + 1;
            xlocation = -(ii-1)*2.04;
            ylocation = (jj-1)*2.04;
            xyzAC(:,idx) = [xlocation;ylocation];
        end
    end
end
%%%CG location
xcg = mean(xyzAC(1,:));
ycg = mean(xyzAC(2,:));
CG = [xcg;ycg]*ones(1,num_ac);
rcg2ac = xyzAC-CG;

%%% Mass Moment of Inertia Matrix in Body Axis (slug.ft^2)
grav = 9.81;
mass1 = 55.07/grav;
mass = num_ac*mass1;

Ix1 = 0.4497;
Iy1 = 0.5111;
Iz1 = 0.8470;
I_b1 = [Ix1 0 0;0 Iy1 0;0 0 Iz1];
%%%Create Parrallel Axis theorem for all of them
I_b = zeros(3,3);
for ii = 1:num_ac
    s = [0 0 rcg2ac(2,ii);0 0 -rcg2ac(1,ii);-rcg2ac(2,ii) rcg2ac(1,ii) 0];
    I_cg = I_b1 + s*s'*mass1;
    I_b = I_b + I_cg;
end

I_b_inv = inv(I_b);

