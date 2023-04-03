for ii = 2:5
    command = ['cp building1.mtl building',num2str(ii),'.mtl'];
    system(command);
    command = ['cp building1.obj building',num2str(ii),'.obj'];
    system(command);
end
