% USPOPENING   Applies the upper skeleton path opening.
%
%      USPOPENING(in,l,h,recons) returns the input image opened using the
%      skeletonized path opening.
%      in - input image
%      l - length of the structuring element.
%      h - height of the h-minima transform.
%      recons - boolean indicating whether reconstruction is desired
%
% In some cases, it may be desirable to skip the initial skeletonization 
% (e.g., if the image contains a lot of zeros). Instead, one may then 
% immediately apply the path opening to the image:
%
% opened = pathOpeningUnbiased(image, length);
%
% where image is an 8-bit grayscale image, and length is the length of the
% structuring element (double).
%
% Note that the input image is expected to have a black border (i.e. value
% 0).
%
% Author:
% Teo Asplund
%
% Reference: A faster, Unbiased Path Opening by Upper Skeletonization and 
% Weighted Adjacency Graphs.
%


function out = uspopening(in,l,h,recons)

if nargin < 3
   h = 0;
   recons = true;
end

if nargin < 4
    recons = true;
end

% Checks
if nargin < 2, error('Needs 2 parameters.'), end
in = uint8(in);
if ndims(in) ~= 2, error('Only for 2D images.'), end

hmin_img = dip_image(imhmin(in, h, 8));
[skeleton, ~] = dip_upperskeleton2d(hmin_img);
skeletonized = uint8(skeleton*in);


out = pathOpeningUnbiased(skeletonized, l);
out = dip_image(out);

if(recons)
    out = reconstruction(out,in,2);
end
