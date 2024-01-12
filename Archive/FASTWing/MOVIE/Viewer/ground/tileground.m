purge

system('rm newground.obj');

fid = fopen('newground.obj','wb');

%%%Output header
fprintf(fid,'%s \n','# Blender3D v249 OBJ File: ground.blend');
fprintf(fid,'%s \n','# www.blender3d.org');
fprintf(fid,'%s \n','mtllib ground.mtl');

%%Output first tile
vertices = [-1 -1 0;1 -1 0;1 1 0;-1 1 0];
for ii = 1:4
  fprintf(fid,'%s ','v ');
  fprintf(fid,'%2.1f %2.1f %2.1f \n',[vertices(ii,1);vertices(ii,2);vertices(ii,3)]);
end
%%Output extra tiles
NUMTILES = 25;
direction = [2 0 0;0 2 0;-2 0 0;0 -2 0];
dir = 0;
counter = 1;
figure()
hold on
rectangle('Position',[vertices(1,1) vertices(1,2) 2 2])
numberofmoves = 0;
for ii = 2:NUMTILES
  if numberofmoves == 0
    numberofmoves = round(counter/2);
    counter = counter + 1;
    dir = dir + 1;
    if dir > 4
      dir = 1;
    end
  end
  %%Output next tiles
  for ii = 1:4
    %%Move tiles in direction 
    vertices(ii,:) = direction(dir,:) + vertices(ii,:);
    fprintf(fid,'%s ','v ');
    fprintf(fid,'%2.1f %2.1f %2.1f \n',[vertices(ii,1);vertices(ii,2);vertices(ii,3)]);
  end
  numberofmoves = numberofmoves - 1;
  rectangle('Position',[vertices(1,1) vertices(1,2) 2 2])
end
%%Output Normals Mtl and S
fprintf(fid,'%s \n','vt 0.000000 0.000000');
fprintf(fid,'%s \n','vt 0.000000 1.000000');
fprintf(fid,'%s \n','vt 1.000000 1.000000');
fprintf(fid,'%s \n','vt 1.000000 0.000000');
fprintf(fid,'%s \n','vn 0.000000 0.00 1.000000');
fprintf(fid,'%s \n','usemtl None_MegaTerrain.jpg');
fprintf(fid,'s off \n');
%%Output Face Coordinates
for ii = 1:NUMTILES
  %%v/vt/vn
  % f 1/1/1 2/2/1 3/3/1
  % f 1/1/1 3/3/1 4/4/1
  face = [1 2 3]+(ii-1)*4;
  fprintf(fid,'f %d/1/1 %d/2/1 %d/3/1 \n',face(1),face(2),face(3));
  face = [1 3 4]+(ii-1)*4;
  fprintf(fid,'f %d/1/1 %d/3/1 %d/4/1 \n',face(1),face(2),face(3));
end

system('cat newground.obj');