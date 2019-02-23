function mask_2d = mask_convert_1d_to_2d(mask_1d)
    X = 640;
    Y = 480;
    
    mask_2d = ones(X*Y, 1);
    mask_2d(mask_1d, 1) = 0;
    mask_2d = reshape(mask_2d, X, Y);
end