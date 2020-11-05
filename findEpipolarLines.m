function [EpipolarLines1, EpipolarLines2] = findEpipolarLines(...
    worldCoord3DPoints, cam1, cam1PixelCoords, cam2, cam2PixelCoords)
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';

% get viewing ray for cam1 and cam2
Rmat1 = cam1.Rmat;
Rtranspose1 = transpose(Rmat1);
Kmat1 = cam1.Kmat;
Kinverse1 = inv(Kmat1);
cam1position = cam1.position;
T1 = [cam1position(1); cam1position(2); cam1position(3)];

u1 = cam1PixelCoords(1,1);
v1 = cam1PixelCoords(2,1);
Pu1 = [u1;v1;1];

v1 = Rtranspose1 * Kinverse1 * Pu1;
c1 = Rtranspose1 * T1;
vUnit1 = v1 ./ norm(v1);

Pw1 = v1 + c1;

% plot points2D2 and points2D4 onto frame
vue2Video = VideoReader(filenamevue2mp4);
vue2Video.CurrentTime = (700-1)*(50/100)/vue2Video.FrameRate;
vid2Frame = readFrame(vue2Video);


figure(700);
set(gcf, 'Position',  [100, 100, 1000, 400])

subplot(1,2,1)
title('vue2');
image(vid2Frame);
axis on;
hold on;
plot(Pw1(1), Pw1(2), '.', 'MarkerSize', 15, 'LineWidth', 2);
title('vue2');
% end plotting of points
end