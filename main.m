function point2D = main()
load 'Subject4-Session3-Take4_mocapJoints.mat' mocapJoints
load 'vue2CalibInfo.mat' vue2
load 'vue4CalibInfo.mat' vue4

%input and parsing of mocap dataset
for mocapFnum = 1000:1001 %size(mocapJoints, 1)
    conf = mocapJoints(mocapFnum,:,4); %confidence values
    s = sum(conf); %sum should equal 12
    if s == 12
        x = mocapJoints(mocapFnum,:,1); %array of 12 X coordinates
        y = mocapJoints(mocapFnum,:,2); % Y coordinates
        z = mocapJoints(mocapFnum,:,3); % Z coordinates (joint coords)
        % need to get the camera parameters here to figure out focalLen
        point2D = project3DpointInto2Dpixel(z, mocapFnum, 1557.8);
    end
end
%input and parsing of camera parameters

end %main