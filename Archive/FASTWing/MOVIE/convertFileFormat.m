purge

infile = 'Input_Files/Meta.OUT';
%%infile = [time body1(quat) body2(quat) ... bodyN(quat) then derivatives]
outfile= 'Input_Files/MetaWinds_MOVIENEW.OUT';
%outfile = [filelength
%time body1(xyz) body2(xyz) ... bodyN(xyz) body1(ptp) body2(ptp) ... bodyN(ptp)]
NBODIES = 2;
NSTATES = 15;
indata = dlmread(infile);
[r,c] = size(indata);
disp(['Filelength = ',num2str(r)])
time = indata(:,1);
indata = indata(:,2:end);
outxyz = zeros(r,3*NBODIES);
outptp = zeros(r,3*NBODIES);
for ii = 1:NBODIES
  idx = (ii-1)*NSTATES;
  xyz = indata(:,1+idx:3+idx);
  ptp = indata(:,4+idx:6+idx);
  %quat = indata(:,4+idx:7+idx);
  %ptp = quat2euler(quat);
  jdx = (ii-1)*3;
  outxyz(:,1+jdx:3+jdx) = xyz;
  outptp(:,1+jdx:3+jdx) = ptp;
end

outdata = [time,outxyz,outptp];

dlmwrite(outfile,outdata,'delimiter',' ');

