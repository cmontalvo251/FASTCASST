purge

system('rm Run.exe');
system('~/Dropbox/BlackBox/compilec source/MMACSV10.cpp -w -O1');
system('./Run.exe source/Meta.ifiles');

files = {'Dummy.txt'};
data = dlmread(['source/Out_Files/Linearized/',files{1}]);
[r,c] = size(data);
Alinear = data(1:c,1:c);
[r,c] = size(Alinear);
NSTATE = 15;
num_ac = round(r/NSTATE);
Blinear = data(NSTATE*num_ac+1:end,1:num_ac*3);
Alinear = Alinear(1:(num_ac*NSTATE),1:(num_ac*NSTATE));

delrows = [];
angles = [];
for jj = 1:num_ac
  s = (jj-1)*NSTATE;
  s2 = (jj-1)*12;
  delrows = [delrows,(s+13):(s+12+3)];
  angles = [angles,(s2+4):(s2+6)];
end
Alinear(delrows,:) =  [];
Alinear(:,delrows) =  [];    1.0000    0         0	  0.0100   0	    -0.0005    0    0.0100   -0    0.0009    0    0    0
         0    1.0000    0         0   	   -0.0164  0    0.1965         0    0.0099         0   -0         0    0
         0    0    	1.0000    0    	   0   	    -0.1965   -0   -0.0009    0    0.0097   -0   -0.0001   -0
         0    0         0    	  1.0000   0        0         0         0         0         0         0         0         0
         0    0         0         0    	   1.0000   0    0         0   -0         0    0.0099         0    0.0009
         0    0         0         0   	   0        1.0000    0    0   -0   -0.0002    0    0.0097    0
         0    0         0         0    	   0        0    1.0000         0    0         0   -0         0    0.0100
         0    0         0         0    	   0   	    -0.0977   -0    0.9992    0    0.0055   -0   -0.0142   -0
         0    0         0         0    	   0.0970   0    0         0    0.9857         0    0.0161         0   -0.1887
         0    0         0         0   	   0   	    -0.0080    0   -0.0049   -0    0.9446    0    0.1747    0
         0    0         0         0   	   -0.0004  0    0         0   -0.0088         0    0.9828         0    0.0166
         0    0         0         0   	   0   	    0    0    0.0038   -0   -0.0413    0    0.9399    0
         0    0         0         0    	   0        0   -0         0    0.0007         0   -0.0040         0    0.9954

Blinear(delrows,:) = [];
A = Alinear
B = Blinear

%%First add a column of zeros
l = 12;
A = [A(:,1:3),zeros(l,1),A(:,4:end)];
%%Now add a row of zeros
A = [A(1:3,:);zeros(1,l+1);A(4:end,:)];
%Finally change A(1,4) = 1 to account for u0
A(1,4) = 1
%%For B we need to simply add a row of zeros
B = [zeros(1,3);B]

timestepDiscrete = 0.001;
Ad = expm(A*timestepDiscrete);
jump = 10;
Adjump = Ad^jump
Ap = A(8:13,8:13);
Adp = Ad(8:13,8:13);
Bp = B(8:13,:);
Bdp = inv(Ap)*(Adp-eye(6))*Bp;
Bdpjump = zeros(6,3);
for ii = 1:jump
  Bdpjump = Bdpjump + Adp^(ii-1)*Bdp;
end
Bdjump = [zeros(7,3);Bdpjump]