function [EpipolarLines1, EpipolarLines2] = findEpipolarLines(...
    worldCoord3DPoints, cam1, cam1PixelCoords, cam2, cam2PixelCoords)

    for i = 1:12
        x1(i) = cam1PixelCoords(1, i);
    end

    for i = 1:12
        y1(i) = cam1PixelCoords(2, i);
    end

    for i = 1:12
        x2(i) = cam2PixelCoords(1, i);
    end

    for i = 1:12
        y2(i) = cam2PixelCoords(2, i);
    end       

        
    %do Hartley preconditioning
    savx1 = x1; savy1 = y1; savx2 = x2; savy2 = y2;
    mux = mean(x1);
    muy = mean(y1);
    stdxy = (std(x1)+std(y1))/2;

    T1 = [1 0 -mux; 0 1 -muy; 0 0 stdxy]/stdxy;
    nx1 = (x1-mux)/stdxy;
    ny1 = (y1-muy)/stdxy;
    mux = mean(x2);
    muy = mean(y2);
    stdxy = (std(x2)+std(y2))/2;
    T2 = [1 0 -mux; 0 1 -muy; 0 0 stdxy]/stdxy;
    nx2 = (x2-mux)/stdxy;
    ny2 = (y2-muy)/stdxy;

    A = [];

    for i=1:length(nx1)
        A(i,:) = [nx1(i)*nx2(i) nx1(i)*ny2(i) nx1(i) ny1(i)*nx2(i) ny1(i)*ny2(i) ny1(i) nx2(i) ny2(i) 1];
    end
    
    [u,d] = eigs(A' * A,1,'SM');
    F = reshape(u,3,3);

    %make F rank 2
    oldF = F;
    [U,D,V] = svd(F);
    D(3,3) = 0;
    F = U * D * V';
    F = T2' * F * T1;

    EpipolarLines1 = F * [x1; y1; ones(size(x1))];
    EpipolarLines2 = ([x2; y2; ones(size(x2))]'*F)';

end
