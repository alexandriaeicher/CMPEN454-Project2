function [points2, points4] = project3DpointInto2Dpixel(X, Y, Z, frame)
    % This function takes a single 3D point and projects it into a 2D pixel
    % location while ignoring the nonlinear distortion parameters in the 
    % "radial" field.
    load 'vue2CalibInfo.mat' vue2
    load 'vue4CalibInfo.mat' vue4
    filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
    filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';

    % Get Kmat and Pmat from vue
    Kmat2 = vue2.Kmat;
    Pmat2 = vue2.Pmat;
    Kmat4 = vue4.Kmat;
    Pmat4 = vue4.Pmat;
    
    % create a 4x1 matrix using the 3D coordinates
    UVW = [X;Y;Z;ones(1,12)];

    % Get the x, y, and z image points
    XYZ2 = Kmat2 * Pmat2 * UVW;
    XYZ4 = Kmat4 * Pmat4 * UVW;

    %initialization of VideoReader for the vue2 video.
    vue2Video = VideoReader(filenamevue2mp4);
    vue4Video = VideoReader(filenamevue4mp4);
    vue2Video.CurrentTime = (frame-1)*(50/100)/vue2Video.FrameRate;
    vue4Video.CurrentTime = (frame-1)*(50/100)/vue4Video.FrameRate;
    vid2Frame = readFrame(vue2Video);
    vid4Frame = readFrame(vue4Video);
    figure(frame);
    set(gcf, 'Position',  [100, 100, 1000, 400])

    x2 = zeros(0, 12);
    y2 = zeros(0, 12);
    x4 = zeros(0, 12);
    y4 = zeros(0, 12);

    % divide each x and y by z to get x, y coordinates
    for i=1:12
        x2(i) = XYZ2(1,i) ./ XYZ2(3,i);
        y2(i) = XYZ2(2,i) ./ XYZ2(3,i);
        x4(i) = XYZ4(1,i) ./ XYZ4(3,i);
        y4(i) = XYZ4(2,i) ./ XYZ4(3,i);
    end
    % plot coordinates on image
    subplot(1,2,1)
    image(vid2Frame);
    axis on;
    hold on;
    plot(x2, y2, '.', 'MarkerSize', 15, 'LineWidth', 2);
    
    subplot(1,2,2);
    image(vid4Frame);
    axis on;
    hold on;
    plot(x4, y4, '.', 'MarkerSize', 15, 'LineWidth', 2);
    
    % return x,y coordinates for frame
    points2 = [x2;y2];
    points4 = [x4,y4];
end
