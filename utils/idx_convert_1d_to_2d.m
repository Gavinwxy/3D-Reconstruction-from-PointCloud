function [x, y] = idx_convert_1d_to_2d(idx_1d)
% Convert 1D index to 2D index
% Args: 
%   idx_1d: a 1D index for a point in point cloud
% Returns:
%   [x, y]: a 2D index for the same point in point cloud

    X = 640;
    
    y = floor(idx_1d / X) + 1;
    x = mod(idx_1d, X);
end