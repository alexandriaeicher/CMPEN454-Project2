function [newPoint] = project3DpointInto2Dpixel(point, frame, focalLen)
% This function takes a single 3D point and projects it into a 2D pixel
% location while ignoring the nonlinear distortion parameters in the 
% "radial" field.

X = point(1);
Y = point(2);
Z = point(3);

x = focalLen * (X ./ Z);
y = focalLen * (Y ./ Z);

newPoint = [x y focalLen];

%For verification, visualize your projected 2D joint by overlaying it as
% a point on the 2D video frame corresponding to the motion capture frame.

%initialization of VideoReader for the vue video.
%YOU ONLY NEED TO DO THIS ONCE AT THE BEGINNING
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
vue2video = VideoReader(filenamevue2mp4);
%now we can read in the video for any mocap frame mocapFnum.
%the (50/100) factor is here to account for the difference in frame
%rates between video (50 fps) and mocap (100 fps).
vue2video.CurrentTime = (frame-1)*(50/100)/vue2video.FrameRate;
vid2Frame = readFrame(vue2video);
image(vid2Frame);
end
