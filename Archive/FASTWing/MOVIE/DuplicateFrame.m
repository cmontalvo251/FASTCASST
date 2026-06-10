Frame = 'Frames/Movie_2169.bmp';

A = imread(Frame);

D = 200;

filename = 'Frames/Movie_';
num = '2169';
add = '0001';
for ii = 1:D
  num = addition(num,add);
  imwrite(A,[filename,num,'.bmp'],'bmp');
end
