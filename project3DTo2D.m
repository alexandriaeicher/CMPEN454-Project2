function projected2DPoints = project3DTo2D(cam, worldCoord3DPoints)
    % inputs a 3x12 matrix of 3D world coordinates
    % outputs a 3x12 matrix of 2D pixel coordinates
    % Get Kmat and Pmat from vue
    Kmat = cam.Kmat;
    Pmat = cam.Pmat;
    
    % create a 4x1 matrix using the 3D coordinates
    worldCoord3DPoints = [worldCoord3DPoints;ones(1,12)];
    
    % Get the x, y, and z image points
    points = Kmat * Pmat * worldCoord3DPoints;
    
    x = zeros(0, 12);
    y = zeros(0, 12);
    
    % divide each x and y by z to get x, y coordinates
    for i=1:12
        x(i) = points(1,i) ./ points(3,i);
        y(i) = points(2,i) ./ points(3,i);
    end
    
    % return x,y coordinates for frame
    projected2DPoints = [x;y;ones(1,12)];
    
end %project3DTo2D