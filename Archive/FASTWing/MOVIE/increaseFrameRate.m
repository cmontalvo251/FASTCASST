root = 'State_Histories/';

file = 'MetaWinds_MOVIE.OUT';
fileout = 'MetaWinds_MOVIE#.OUT';

factor = 10;
data = dlmread([root,file]);
[r,c] = size(data);
timestep = data(2,1);
dt = timestep/factor;
newdata = zeros(r*10,c);
counter = 1;
for ii = 1:r-1
  prev = data(ii,:);
  next = data(ii+1,:);
  slope = (next-prev)./timestep;
  for jj = 1:factor
    newdata(counter,:) = slope.*((jj-1)*dt)+prev;
    counter = counter + 1;
  end
end

disp(['New Size = ',num2str(r*factor)])

dlmwrite([root,fileout],newdata,'delimiter',' ')