rng(0);

addpath('./sift_match_transform');
pcs = load('dataset/office1.mat');
pcs = pcs.pcl_train;
mask_collection = load('mask_collection.mat');
masks = mask_collection.masks;

frame = 13; % frame from 2 to 40

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

for i = 1:size(valid_pairs, 2)
    color_pc1(sift_pairs(1,i), :) = [0, 255, 0];
    color_pc2(sift_pairs(2,i), :) = [0, 255, 0];
end

invalid_pairs = sift_pairs;
invalid_pairs(:,pt_idx) = [];

for i = 1:size(invalid_pairs, 2)
    color_pc1(invalid_pairs(1,i), :) = [255, 0, 0];
    color_pc2(invalid_pairs(2,i), :) = [255, 0, 0];
end

figure(1);
imshow(imag2d(color_pc1))
figure(2);
imshow(imag2d(color_pc2))

% Merge two point clouds
mask1 = masks{frame1};
mask2 = masks{frame2};

loc_pc1 = pc1.Location;
loc_pc1(mask1,:) = [];
loc_pc2 = pc2.Location;
loc_pc2(mask2,:) = [];

color_pc1 = pc1.Color;
color_pc1(mask1,:) = [];
color_pc2 = pc2.Color;
color_pc2(mask2,:) = [];

loc_pc1 = cat(2, loc_pc1, ones(size(loc_pc1, 1), 1));
loc_pc1 = (model*loc_pc1')';
loc_pc1 = loc_pc1(:,1:3);

loc_pc = cat(1, loc_pc1, loc_pc2);
color_pc = cat(1, color_pc1, color_pc2);

pc = pointCloud(loc_pc, 'Color', color_pc);
figure(3);
pcshow(pc)