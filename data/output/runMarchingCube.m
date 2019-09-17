% Compute implicit surface
clear variables;

load("For mesh.mat")

x = FormeshP(:,1);
y = FormeshP(:,2);
z = FormeshP(:,3);
mean = FormeshM;
var = FormeshV;

isize = 100;
nxx = isize; nyy = isize; nzz = isize;
xq = linspace(min(x), max(x), isize); 
yq = linspace(min(y), max(y), isize);
zq = linspace(min(z), max(z), isize);
[Xq, Yq, Zq] = meshgrid(xq, yq, zq);

MEANq = griddata(x,y,z,mean,Xq,Yq,Zq,'linear');
VARq = griddata(x,y,z,var,Xq,Yq,Zq,'linear');

[faces,verts,variance_on_surface] = isosurface(Xq, Yq, Zq, MEANq, 0, VARq);
%[faces,verts,colors] = MarchingCubes(Xq, Yq, Zq, Vq, 0);

figure();
patch('Vertices',verts,'Faces',faces,'FaceVertexCData',variance_on_surface,...
    'FaceColor','interp','EdgeColor','interp')
view(30,-15)
axis vis3d
daspect([1 1 1])
view(3); 
axis tight
camlight 
lighting gouraud
colormap parula

writeOBJ("bunny.obj", verts,faces);
csvwrite("variance.csv",variance_on_surface);