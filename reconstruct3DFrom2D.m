function recovered3DPoints = reconstruct3DFrom2D(cam1, cam1PixelCoords, cam2, cam2PixelCoords)

% get viewing ray for cam1 and cam2
Rmat1 = cam1.Rmat;
Rtranspose1 = transpose(Rmat1);
Kmat1 = cam1.Kmat;
Kinverse1 = inv(Kmat1);
cam1position = cam1.position;
T1 = [cam1position(1); cam1position(2); cam1position(3)];

Rmat2 = cam2.Rmat;
Rtranspose2 = transpose(Rmat2);
Kmat2 = cam2.Kmat;
Kinverse2 = inv(Kmat2);
cam2position = cam2.position;
T2 = [cam2position(1); cam2position(2); cam2position(3)];

u1 = cam1PixelCoords(1,1);
v1 = cam1PixelCoords(2,1);
Pu1 = [u1;v1;1];

v1 = Rtranspose1 * Kinverse1 * Pu1;
c1 = Rtranspose1 * T1;
vUnit1 = v1 ./ norm(v1);

u2 = cam2PixelCoords(1,1);
v2 = cam2PixelCoords(2,1);
Pu2 = [u2;v2;1];

v2 = Rtranspose2 * Kinverse2 * Pu2;
c2 = Rtranspose2 * T2;
vUnit2 = v2 ./ norm(v2);


end