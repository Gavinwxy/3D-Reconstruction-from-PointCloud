rng(0);

addpath('./2_valid_sift_match');
addpath('./utils')
pcs = load('dataset/office1.mat');
pcs = pcs.pcl_train;
mask_collection = load('results/mask_collection.mat');
masks = mask_collection.masks;

frame = 20; % frame >= 2

frame1 = frame;
frame2 = frame-1;

pc1 = pcs{frame1};
pc2 = pcs{frame2};

mask1 = mask_convert_1d_to_2d(masks{frame1});
mask2 = mask_convert_1d_to_2d(masks{frame2});

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

% Difficult frames:
if frame == 23
    sift_dist_th = 90;
    best_2nd_ratio = 0.8;
    ransac_param.sample_size = 3;
elseif frame == 25
    sift_dist_th = 100;
    best_2nd_ratio = 0.9;
    ransac_param.sample_size = 3;
    ransac_param.th_dist = 0.5;
elseif frame == 27
    best_2nd_ratio = 0.95;
    ransac_param.sample_size = 3;
    ransac_param.th_dist = 0.5;
    ransac_param.inl_ratio = 0.1;
elseif frame == 28
    best_2nd_ratio = 0.8;
    ransac_param.sample_size = 3;
    ransac_param.th_dist = 0.5;
    ransac_param.inl_ratio = 0.1;
elseif frame == 30
    sift_dist_th = 70;
elseif frame == 31
    sift_dist_th = 90;
    ransac_param.th_dist = 0.3;
elseif frame == 32
    sift_dist_th = 90;
    best_2nd_ratio = 0.9;
    ransac_param.th_dist = 0.2;
    ransac_param.inl_ratio = 0.1;
end

sift_pairs = valid_sift(rgb_img1, mask1, rgb_img2, mask2, sift_dist_th, area_ratio_th, best_2nd_ratio);
[A, B] = get_depth(pc1, pc2, sift_pairs);
[model, pt_idx] = ransac_icp(A, B, ransac_param);

% Display matched and mismatched points in images
color_pc1 = pc1.Color;
color_pc2 = pc2.Color;

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

figure; ax = axes;
showMatchedFeatures(rgb_img1,rgb_img2,v_left_coord,v_right_coord,'montage','Parent',ax);


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

figure; ax = axes;
showMatchedFeatures(rgb_img1,rgb_img2,inv_left_coord,inv_right_coord,'montage','Parent',ax);