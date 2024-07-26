%%CombinesFrames
purge

folder1 = 'Frames_Connected/';
folder2 = 'FramesNot_Connected/';
folder3 = 'FramesCombined/';

for ii = 20350:20792
  filename = ['Movie_',num2str(ii),'.bmp']
  image1 = imread([folder1,filename]);
  image2 = imread([folder2,filename]);
  % imshow(image1)
  % pause
  % imshow(image2)
  image3 = [image1,image2];
  %pause
  %close all
  %imshow(image3)
  imwrite(image3,[folder3,filename])
  clear image1 image2 image3
end
