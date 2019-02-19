function imag2d(rgb)
    new_rgb =[];
    color_pc = rgb;

    %% Extracting the r, g, b colours
    r = color_pc(:,1);
    g = color_pc(:,2);
    b = color_pc(:,3);

    %% reshaping each array (r, g, b) to obtain a [512x424] matrix 
    rec_r = reshape(r, [640, 480]);
    rec_g = reshape(g, [640, 480]);
    rec_b = reshape(b, [640, 480]);
    new_rgb = cat(3, rec_r', rec_g', rec_b');
    imshow(new_rgb)
    pause(0.1)
end