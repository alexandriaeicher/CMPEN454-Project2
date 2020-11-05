function worldCoord3DPoints = main()
%points2D2 = main()
  
load 'Subject4-Session3-Take4_mocapJoints.mat' mocapJoints
load 'vue2CalibInfo.mat' vue2
load 'vue4CalibInfo.mat' vue4
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2.mp4';
filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4.mp4';
    
vue2Video = VideoReader(filenamevue2mp4);
vue4Video = VideoReader(filenamevue4mp4);

%input and parsing of mocap dataset
for mocapFnum = 700:700 %size(mocapJoints, 1)
    conf = mocapJoints(mocapFnum,:,4); %confidence values
    s = sum(conf); %sum should equal 12
    if s == 12
        x = mocapJoints(mocapFnum,:,1); %array of 12 X coordinates
        y = mocapJoints(mocapFnum,:,2); % Y coordinates
        z = mocapJoints(mocapFnum,:,3); % Z coordinates (joint coords)
        worldCoord3DPoints = [x;y;z];
        
        % project3DTo2D
        points2D2 = project3DTo2D(vue2, worldCoord3DPoints);
        points2D4 = project3DTo2D(vue4, worldCoord3DPoints);
        
        % plot points2D2 and points2D4 onto frame
%         vue2Video.CurrentTime = (mocapFnum-1)*(50/100)/vue2Video.FrameRate;
%         vid2Frame = readFrame(vue2Video);
%         vue4Video.CurrentTime = (mocapFnum-1)*(50/100)/vue4Video.FrameRate;
%         vid4Frame = readFrame(vue4Video);
%         
%         figure(mocapFnum);
%         set(gcf, 'Position',  [100, 100, 1000, 400])
%         
%         subplot(1,2,1)
%         title('vue2');
%         image(vid2Frame);
%         axis on;
%         hold on;
%         plot(points2D2(1,:,:), points2D2(2,:,:), '.', 'MarkerSize', 15, 'LineWidth', 2);
%         title('vue2');
%         
%         subplot(1,2,2);
%         image(vid4Frame);
%         axis on;
%         hold on;
%         plot(points2D4(1,:,:), points2D4(2,:,:), '.', 'MarkerSize', 15, 'LineWidth', 2);
%         title('vue4');
        % end plotting of points
        
        % recostruct3DFrom2D
        recovered3DPoints = reconstruct3DFrom2D(vue2, points2D2, vue4, points2D4);
        % findEpipolarLines(worldCoord3DPoints, vue2, points2D2, vue4, points2D4);
        
    end
end

end %main
