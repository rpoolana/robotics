
% Read the image
img0 = imread('img05.jpg');
h = fspecial('gaussian',[3 3],0.5);

% constants
k = 0.04;
e = 2.71828;
Threshold = 5*e - 4;
sigma = 3;

% Image filter
img0 = myImageFilter(img0,h);
imwrite(img0, 'image_filter_out_05.png');

% edge detector
[Im, Io, Ix, Iy] = myEdgeFilter(img0, 0.5);
imwrite(Im, 'edge_filter_out_05.png');

% Corner detector
cornerness = myHarrisCorner(Ix,Iy,Threshold);

% add circles around the corners
[rows,cols]=size(cornerness);

for i=1:rows
    for j=1:cols
        if cornerness(i,j)>0
            img0(i,j,1)=255;
        end
    end
end

imwrite(img0, 'corner_detector_out_05.png');

            












