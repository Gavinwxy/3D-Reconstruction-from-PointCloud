%% Set seed and add paths
rng(0);

addpath('./1_preprocessing');
addpath('./2_valid_sift_match');
addpath('./3_ransac_solve');
addpath('./4_pc_fusion');
addpath('./5_evaluation');
addpath('./dataset');
addpath('./results');
addpath('./utils');

%% Load data
pcs = load('dataset/office1.mat');
pcs = pcs.pcl_train;

mask_collection = load('results/mask_collection.mat');
masks = mask_collection.masks;

new_office = load('results/new_office.mat');
transformed_pcs = new_office.transformed_pcs;

%% Begin visualization
pc_merged = transformed_pcs{1};

for frame=2:size(masks, 2)
    frame1 = frame;
    frame2 = frame-1;

    pc1 = pcs{frame1};
    pc2 = pcs{frame2};
    
    mask1_linear = masks{frame1};
    mask2_linear = masks{frame2};

    mask1 = mask_convert_1d_to_2d(mask1_linear);
    mask2 = mask_convert_1d_to_2d(mask2_linear);

    rgb_img1 = imag2d(pc1.Color);
    rgb_img2 = imag2d(pc2.Color);

    % Hyper parameters
    sift_dist_th = 60; % the less the stricter
    area_ratio_th = 0.9; % the more the stricter
    best_2nd_ratio = 0.7; % the less the stricter
    ransac_param.sample_size = 20; % the more the stricter
    ransac_param.th_dist = 0.1; % the less the stricter
    ransac_param.itr_num = 100; % number of iteration
    ransac_param.inl_ratio = 0.2; % the more the stricter

    sift_pairs = valid_sift(rgb_img1, mask1, rgb_img2, mask2, sift_dist_th, area_ratio_th, best_2nd_ratio);
    [A, B] = get_depth(pc1, pc2, sift_pairs);
    [model, pt_idx] = ransac_icp(A, B, ransac_param);
    
    % The original image and point cloud, the cleand point cloud:
    figure(1);
    imshow(rgb_img1);
    figure(2);
    pcshow(pc1);

    pc = pc1;
    color = pc.Color;
    loc = pc.Location;
    color(mask1_linear,:) = 0;
    loc(mask1_linear,:) = 0;
    pc = pointCloud(loc, 'Color', color);
    figure(3);
    pcshow(pc);
    
    % Binary image for mask:
    figure(4);
    imshow(mask1');

    % Display matched and mismatched points in images:

    valid_pairs = sift_pairs(:,pt_idx);
    valid_pairs_left = valid_pairs(1, :)';
    v_left_coord = [];
    for i=1:1:size(valid_pairs_left, 1)
        [x,y] = idx_convert_1d_to_2d(valid_pairs_left(i));
        v_left_coord = [v_left_coord; [x, y]];
    end

    valid_pairs_right = valid_pairs(2, :)';
    v_right_coord = [];
    for i=1:1:size(valid_pairs_right, 1)
        [x,y] = idx_convert_1d_to_2d(valid_pairs_right(i));
        v_right_coord = [v_right_coord; [x, y]];
    end

    figure(6);
    showMatchedFeatures(rgb_img2,rgb_img1,v_right_coord,v_left_coord,'montage');


    invalid_pairs = sift_pairs;
    invalid_pairs(:,pt_idx) = [];

    invalid_pairs_left = invalid_pairs(1, :)';
    inv_left_coord = [];
    for i=1:1:size(invalid_pairs_left, 1)
        [x,y] = idx_convert_1d_to_2d(invalid_pairs_left(i));
        inv_left_coord = [inv_left_coord; [x, y]];
    end

    invalid_pairs_right = invalid_pairs(2, :)';
    inv_right_coord = [];
    for i=1:1:size(invalid_pairs_right, 1)
        [x,y] = idx_convert_1d_to_2d(invalid_pairs_right(i));
        inv_right_coord = [inv_right_coord; [x, y]];
    end

    figure(7);
    showMatchedFeatures(rgb_img2,rgb_img1,inv_right_coord,inv_left_coord,'montage');

    
    sift_left_coord = [v_left_coord; inv_left_coord];
    figure(5);
    imshow(rgb_img1);
    hold on;
    for i =1:size(sift_left_coord, 1)
        p = sift_left_coord(i,:);
        plot(p(1), p(2), 'r*', 'LineWidth', 2, 'MarkerSize', 3);
    end
    
    two_frames_pc = pcmerge(transformed_pcs{frame2}, transformed_pcs{frame1}, 0.015);
    figure(8);
    pcshow(two_frames_pc);
    
    pc_merged = pcmerge(pc_merged, transformed_pcs{frame}, 0.015);
    figure(9);
    pcshow(pc_merged);
    
    pause;
end