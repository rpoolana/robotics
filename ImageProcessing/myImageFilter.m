% References:
% https://en.wikipedia.org/wiki/Kernel_(image_processing)
% http://www.songho.ca/dsp/convolution/convolution.html

% Applying the convolution algorithm
% for each image row in input image:
%    for each pixel in image row:
%       set accumulator to zero
%       for each kernel row in kernel:
%          for each element in kernel row:
%             if element position  corresponding* to pixel position then
%                multiply element value  corresponding* to pixel value
%                add result to accumulator
%             endif
%       set output image pixel to accumulator

% reads image0 convolutes with filter h and returns the filtered image
function img1 = myImageFilter(img0,h)

% converts the intensity image I to single
img0 = im2single(img0);
h = im2single(h);

% get the size of the input image
[inpRows, inpCols]=size(img0);

% Get the center of the kernel
[hRows, hCols] = size(h);
centerHX = floor(hRows/2);
centerHY = floor(hCols/2);

% create output image matrix of the dimensions of the input image
img1 = zeros(inpRows, inpCols);

for i=1 : inpRows
    for j=1 : inpCols
        % initialize the value that will hold the weighted sum
        sum = 0;
        for k=1:hRows
              kPrime = hRows - k;
            for l=1:hCols
                lPrime = hCols - l;                
                % index of input signal, used for checking boundary
                iPrime = i + (k - centerHX);
                jPrime = j + (l - centerHY);
                % Out-of-bound values will not be considered
                if ((iPrime >= 1 && iPrime < inpRows) && (jPrime >= 1 && jPrime < inpCols) && (kPrime>0) && (lPrime>0)) 
                    sum = sum + (img0(iPrime,jPrime)* h(kPrime,lPrime));                
                end
            end
            img1(i,j) = sum;
        end       
    end
end