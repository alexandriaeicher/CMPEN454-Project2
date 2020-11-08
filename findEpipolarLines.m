function [EpipolarLines1, EpipolarLines2] = findEpipolarLines(...
    worldCoord3DPoints, cam1, cam1PixelCoords, cam2, cam2PixelCoords)
filenamevue2mp4 = 'Subject4-Session3-24form-Full-Take4-Vue2_updated.mp4';
filenamevue4mp4 = 'Subject4-Session3-24form-Full-Take4-Vue4_updated.mp4';


x1 = []

for i = 1:12;
    x1(i) = cam1PixelCoords(1, i)
    
end

for i = 1:12;
    y1(i) = cam1PixelCoords(2, i)
    
end

for i = 1:12;
    x2(i) = cam2PixelCoords(1, i)
    
end

for i = 1:12;
    y2(i) = cam2PixelCoords(2, i)
    
end



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
    end        

        
        %do Hartley preconditioning
     
    savx1 = x1; savy1 = y1; savx2 = x2; savy2 = y2;
    mux = mean(x1);
    disp(mux)
    muy = mean(y1);
    disp(muy)
    disp(std(x1))
    stdxy = (std(x1)+std(y1))/2;
    disp(stdxy)
    T1 = [1 0 -mux; 0 1 -muy; 0 0 stdxy]/stdxy;
    nx1 = (x1-mux)/stdxy;
    ny1 = (y1-muy)/stdxy;
    mux = mean(x2);
    muy = mean(y2);
    stdxy = (std(x2)+std(y2))/2;
    T2 = [1 0 -mux; 0 1 -muy; 0 0 stdxy]/stdxy;
    nx2 = (x2-mux)/stdxy;
    ny2 = (y2-muy)/stdxy;
        
     %   A = [];
     %   for i=1:12;
            %disp(cam1PixelCoords(i))
            %disp(cam1PixelCoords(i,2))
     %       A(i,:) = [cam1PixelCoords(1,i)*cam2PixelCoords(1,i) cam1PixelCoords(1,i)*cam2PixelCoords(2,i) cam1PixelCoords(1,i) cam1PixelCoords(2,i)*cam2PixelCoords(1,i) cam1PixelCoords(2,i)*cam2PixelCoords(2,i) cam1PixelCoords(2,i) cam2PixelCoords(1,i) cam2PixelCoords(2,i) cam2PixelCoords(3,i)];
     %   end
     
     A = [];
     disp(nx1)
    for i=1:length(nx1);
        A(i,:) = [nx1(i)*nx2(i) nx1(i)*ny2(i) nx1(i) ny1(i)*nx2(i) ny1(i)*ny2(i) ny1(i) nx2(i) ny2(i) 1];
    end
    
        [u,d] = eigs(A' * A,1,'SM');
        F = reshape(u,3,3);
        
        disp(A)

    %make F rank 2
    oldF = F;
    [U,D,V] = svd(F);
    D(3,3) = 0;
    F = U * D * V';

    F = T2' * F * T1;

     
  disp("hello")
    disp(F)

% plot points2D2 and points2D4 onto frame
vue2Video = VideoReader(filenamevue2mp4);
vue4Video = VideoReader(filenamevue4mp4);
vue2Video.CurrentTime = (700-1)*(50/100)/vue2Video.FrameRate;
vue4Video.CurrentTime = (700-1)*(50/100)/vue4Video.FrameRate;
vid2Frame = readFrame(vue2Video);
vid4Frame = readFrame(vue4Video);


colors =  'bgrcmykbgrcmykbgrcmykbgrcmykbgrcmykbgrcmykbgrcmyk';

figure(700);
set(gcf, 'Position',  [100, 100, 1000, 400])

EpipolarLines1 = F * [x1; y1; ones(size(x1))];
EpipolarLines2 = F * [x2; y2; ones(size(x2))];

[nr,nc,nb] = size(vid2Frame);

L = F * [x1; y1; ones(size(x1))];


%subplot(1,2,1)
%         title('vue2');
%         image(vid2Frame);

%for i=1:length(L);
%    a = L(1,i); b = L(2,i); c=L(3,i);
%    if (abs(a) > (abs(b)))
%       ylo=0; yhi=nr; 
%       xlo = (-b * ylo - c) / a;
%       xhi = (-b * yhi - c) / a;
%       hold on
%       h=plot([xlo; xhi],[ylo; yhi]);
%       set(h,'Color',colors(i),'LineWidth',2);
%       hold off
%       drawnow;
%    else
%       xlo=0; xhi=nc; 
%       ylo = (-a * xlo - c) / b;
%       yhi = (-a * xhi - c) / b;
%       hold on
%       h=plot([xlo; xhi],[ylo; yhi],'b');
%       set(h,'Color',colors(i),'LineWidth',2);
%       hold off
%       drawnow;
%    end
%end

%L = F * [x2; y2; ones(size(x2))];
%subplot(1,2,1)
%         title('vue4');
%         image(vid4Frame);
%for i=1:length(L)
%    a = L(1,i); b = L(2,i); c=L(3,i);
%    if (abs(a) > (abs(b)))
%       ylo=0; yhi=nr; 
%       xlo = (-b * ylo - c) / a;
%       xhi = (-b * yhi - c) / a;
%       hold on
%       h=plot([xlo; xhi],[ylo; yhi],'b');
%       set(h,'Color',colors(i),'LineWidth',2);
%       hold off
%       drawnow;
%    else
%       xlo=0; xhi=nc; 
%       ylo = (-a * xlo - c) / b;
%       yhi = (-a * xhi - c) / b;
%       hold on
%       h=plot([xlo; xhi],[ylo; yhi],'b');
%       set(h,'Color',colors(i),'LineWidth',2);
%       hold off
%       drawnow;
%    end
%end


end
