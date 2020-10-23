function points2D = main()

load 'Subject4-Session3-Take4_mocapJoints.mat' mocapJoints

%input and parsing of mocap dataset
for mocapFnum = 1000:1002 %size(mocapJoints, 1)
    conf = mocapJoints(mocapFnum,:,4); %confidence values
    s = sum(conf); %sum should equal 12
    if s == 12
        x = mocapJoints(mocapFnum,:,1); %array of 12 X coordinates
        y = mocapJoints(mocapFnum,:,2); % Y coordinates
        z = mocapJoints(mocapFnum,:,3); % Z coordinates (joint coords)
        
        points2D = project3DpointInto2Dpixel(x, y, z, mocapFnum);
    end
end

end %main
