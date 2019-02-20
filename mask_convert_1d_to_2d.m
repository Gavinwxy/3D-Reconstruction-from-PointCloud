function mask_2d = mask_convert_1d_to_2d(mask_1d)
    mask_2d = ones(307200, 1);
    mask_2d(mask_1d, 1) = 0;
    mask_2d = reshape(mask_2d, 640, 480);
end