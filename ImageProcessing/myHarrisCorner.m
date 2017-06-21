% References:
% http://docs.opencv.org/2.4/doc/tutorials/features2d/trackingmotion/harris_detector/harris_detector.html
% http://slazebni.cs.illinois.edu/spring16/harris.m

function [R] = myHarrisCorner(Ix,Iy,Threshold)

k = 0.04;
sigma = 3;
h = fspecial('gaussian',[3 3],0.5);

[x2, y2] = meshgrid(-sigma:sigma, -sigma:sigma);
Fx = x2 .* exp(-(x2 .^ 2 + y2.^ 2) / (2 * sigma ^ 2));
Fy = y2 .* exp(-(x2 .^ 2 + y2 .^ 2) / (2 * sigma ^ 2));

Ix = myImageFilter(Ix, Fx);
Iy = myImageFilter(Iy, Fy);

% Calculate Ix2, Iy2, Ixy and smoothen them with gaussian filter
Ix2 = myImageFilter(Ix .^ 2,h);
Iy2 = myImageFilter(Iy .^ 2,h);
Ixy = myImageFilter(Ix .* Iy,h);

R0 = (Ix2.*Iy2 - Ixy.^2) - k*(Ix2 + Iy2).^2;
R = zeros(size(R0));

[rows,cols] = size(R0);

for i=1:rows
     for j=1:cols
        value = R0(i,j);
        if value > Threshold
              R(i,j) = R0(i,j);
        end
     end
end

% nonmax suppression
% radius = 3;
% suppress = ordfilt2(R0, radius,ones(3,3),true);
X = imdilate(R, [1 1 1; 1 0 1; 1 1 1]);
R = R > X;
% circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
% figure, imshow(I);
% figure, imshow(output)
imwrite(R,'imgCorner.png')