function idx_1d = idx_convert_2d_to_1d(idx_2d)
% Convert 2D index to 1D index
% Args: 
%   idx_2d: a 2D index for a point in point cloud
% Returns:
%   idx_1d: a 1D index for the same point in point cloud

    X = 640;
    
    x = idx_2d(1);
    y = idx_2d(2);
    idx_1d = (y-1)*X + x;
end