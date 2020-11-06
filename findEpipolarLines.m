function [EpipolarLines1, EpipolarLines2] = findEpipolarLines(...
    worldCoord3DPoints, cam1, cam1PixelCoords, cam2, cam2PixelCoords)
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2_updated.mp4';

disp(cam1PixelCoords)

for i = 1:12
    x1=cam1PixelCoords(1, i)
end

disp(x1)

Rmat1 = cam1.Rmat;
    Rtranspose1 = transpose(Rmat1);
    Kmat1 = cam1.Kmat;
    Kinverse1 = inv(Kmat1);
    cam1position = cam1.position;
    cx1 = cam1position(1);
    cy1 = cam1position(2);
    cz1 = cam1position(3);

    t11 = -((cx1*Rmat1(1,1)) + (cy1*Rmat1(1,2)) + (cz1*Rmat1(1,3)));
    t12 = -((cx1*Rmat1(2,1)) + (cy1*Rmat1(2,2)) + (cz1*Rmat1(2,3)));
    t13 = -((cx1*Rmat1(3,1)) + (cy1*Rmat1(3,2)) + (cz1*Rmat1(3,3)));

    T1 = [t11; t12; t13];
    
     % set up variables to get viewing ray for cam2
    Rmat2 = cam2.Rmat;
    Rtranspose2 = transpose(Rmat2);
    Kmat2 = cam2.Kmat;
    Kinverse2 = inv(Kmat2);
    cam2position = cam2.position;
    cx2 = cam2position(1);
    cy2 = cam2position(2);
    cz2 = cam2position(3);

    t21 = -((cx2*Rmat2(1,1)) + (cy2*Rmat2(1,2)) + (cz2*Rmat2(1,3)));
    t22 = -((cx2*Rmat2(2,1)) + (cy2*Rmat2(2,2)) + (cz2*Rmat2(2,3)));
    t23 = -((cx2*Rmat2(3,1)) + (cy2*Rmat2(3,2)) + (cz2*Rmat2(3,3)));

    T2 = [t21; t22; t23];
    

    for i=1:12
        Pu1 = [cam1PixelCoords(1,i); cam1PixelCoords(2,i); cam1PixelCoords(3,i)];
        V1 = Rtranspose1 * Kinverse1 * Pu1;
        C1 = Rtranspose1 * T1;
        Pw1 = V1 + C1;
        VUnit1 = V1 ./ norm(V1);

        Pu2 = [cam2PixelCoords(1,i); cam2PixelCoords(2,i); cam2PixelCoords(3,i)];
        V2 = Rtranspose2 * Kinverse2 * Pu2;
        C2 = Rtranspose2 * T2;
        Pw2 = V2 + C2;
        VUnit2 = V2 ./ norm(V2);
        

        % use unit vectors to get third unit vector
        V3Cross = cross(VUnit1, VUnit2);
        VUnit3 =  V3Cross / norm(V3Cross);
        
        A = [VUnit1(1) -VUnit2(1) VUnit3(1);...
            VUnit1(2) -VUnit2(2) VUnit3(2);...
            VUnit1(3) -VUnit2(3) VUnit3(3)];
        
         b = [C2(1)-C1(1); C2(2)-C1(2); C2(3)-C1(3)];

        x = A\b;
        a = x(1);
        b = x(2);
        d = x(3);

        % use a, b, and d to find points. Then find the midpoint of the two points.
        % the midpoint is the answer.
        p1 = C1 + a*VUnit1;
        p2 = C2 + b*VUnit2;
        p = (p1 + p2) / 2;

        recovered3DPoints(1,i) = -p(1);
        recovered3DPoints(2,i) = -p(2);
        recovered3DPoints(3,i) = -p(3);
        
        %do Hartley preconditioning
 %   savx1 = x1; savy1 = y1; savx2 = x2; savy2 = y2;
 %   mux = mean(x1);
 %   muy = mean(y1);
 %   stdxy = (std(x1)+std(y1))/2;
 %   T1 = [1 0 -mux; 0 1 -muy; 0 0 stdxy]/stdxy;
 %   nx1 = (x1-mux)/stdxy;
 %   ny1 = (y1-muy)/stdxy;
 %   mux = mean(x2);
 %   muy = mean(y2);
 %   stdxy = (std(x2)+std(y2))/2;
 %   T2 = [1 0 -mux; 0 1 -muy; 0 0 stdxy]/stdxy;
 %   nx2 = (x2-mux)/stdxy;
 %   ny2 = (y2-muy)/stdxy;
        
        A = [];
        for i=1:8;
            %disp(cam1PixelCoords(i))
            %disp(cam1PixelCoords(i,2))
            A(i,:) = [cam1PixelCoords(i)*cam2PixelCoords(i) cam1PixelCoords(i)*cam1PixelCoords(i) cam1PixelCoords(i) cam1PixelCoords(i)*cam2PixelCoords(i) cam1PixelCoords(i)*cam2PixelCoords(i) cam1PixelCoords(i) cam2PixelCoords(i) cam2PixelCoords(i) 1];
        end
        
        [~, ~, V] = svd(A);
        F = reshape(V(:,9), 3, 3)';
        
        
        [U, D, V] = svd(F);
        D(3,3) = 0; %rank 2 constrains
        F = U*D*V';
        
        disp(F)
        
        % rescale fundamental matrix
        F = T2' * F * T1;
        
      
     
        %get eigenvector associated with smallest eigenvalue of A' * A
       % [u,d] = eigs((A' * A), 1,'SM');
       % disp(u)
       % F = reshape(u,3,3);
       % disp(F)
        
        %make F rank 2
     %   oldF = F;
     %   [U,D,V] = svd(F);
     %   D(3,3) = 0;
     %   F = U * D * V';
        
        %unnormalize F to undo the effects of Hartley preconditioning
      %  F = T2' * F * T1;

% plot points2D2 and points2D4 onto frame
vue2Video = VideoReader(filenamevue2mp4);
vue2Video.CurrentTime = (700-1)*(50/100)/vue2Video.FrameRate;
vid2Frame = readFrame(vue2Video);

colors =  'bgrcmykbgrcmykbgrcmykbgrcmykbgrcmykbgrcmykbgrcmyk';

figure(700);
set(gcf, 'Position',  [100, 100, 1000, 400])


subplot(1,2,1)
         title('vue2');
         image(vid2Frame);
         axis on;
         hold on;
         plot(cam1PixelCoords(1,:,:), cam2PixelCoords(2,:,:), '.', 'MarkerSize', 15, 'Color',colors(i), 'LineWidth', 2);
       %  plot(C1, Pw1, '.', 'MarkerSize', 15, 'Color',colors(i), 'LineWidth', 2);
         title('vue2');

%subplot(1,2,1)
%title('vue2');
%image(vid2Frame);
%axis on;
%hold on;
%plot(Pw1(1), Pw1(2), '.', 'MarkerSize', 15, 'LineWidth', 2);
%title('vue2');
% end plotting of points
end
