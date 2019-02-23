function mask_2d = mask_convert_1d_to_2d(mask_1d)
% Convert 1D list of discarded points' indexes to a 2D mask
% Args:
%   mask_1d: a 1D list of discarded points' indexes in a point cloud
% Returns:
%   mask_2d: 2D matrix indicating whether a pixel is valid or discarded

    X = 640;
    Y = 480;
    
    mask_2d = ones(X*Y, 1);
    mask_2d(mask_1d, 1) = 0;
    mask_2d = reshape(mask_2d, X, Y);
end