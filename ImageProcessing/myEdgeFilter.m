% References: 
% http://angeljohnsy.blogspot.com/2011/12/sobel-edge-detection.html
% http://www.peterkovesi.com/matlabfns/Spatial/canny.m

function [Im, Io, Ix, Iy] = myEdgeFilter(img, sigma)

  h = fspecial('gaussian',[3 3],sigma);
  img = myImageFilter(img,h);

  filterX = [-1,0,1;-2,0,2;-1,0,1];
  filterY = [1,2,1;0,0,0;-1,-2,-1];
  
  Ix = myImageFilter(img,filterX);
  Iy = myImageFilter(img,filterY);

  % Calculate image orientation
    Io = atan2(-Iy, Ix);                
    Io(Io<0) = Io(Io<0)+pi;           
    Io = Io*180/pi;     
  
  % Calculate Im
    Im = Ix + Iy;
    
   % sharpen image with radius 1
   Im = imsharpen(Im, 'Radius',1,'Amount',5);
