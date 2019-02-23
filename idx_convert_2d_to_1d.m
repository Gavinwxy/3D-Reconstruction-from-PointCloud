function idx_1d = idx_convert_2d_to_1d(idx_2d)
    X = 640;
    
    x = idx_2d(1);
    y = idx_2d(2);
    idx_1d = (y-1)*X + x;
end