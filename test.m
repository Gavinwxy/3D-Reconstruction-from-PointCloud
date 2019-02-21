frame1 = 17;
frame2 = 18;

addpath('./sift_match_transform');
pc = load('dataset/office1.mat');
pc = pc.pcl_train;
pc1 = pc{frame1};
pc2 = pc{frame2};
a = load('mask_collection.mat');
masks = a.masks;

mask1 = mask_convert_1d_to_2d(masks{frame1});
mask2 = mask_convert_1d_to_2d(masks{frame2});

rgb_img1 = imag2d(pc1.Color);
rgb_img2 = imag2d(pc2.Color);

% Hyper parameters
sift_dist_th = 50;
ratio_th = 0.8;
ransac_param.sample_size = 10; % number of sample points to use
ransac_param.th_dist = 1.5; % distance threshold
ransac_param.itr_num = 100; % number of iteration
ransac_param.inl_ratio = 0.5;% inlier ratio

sift_pairs = valid_sift(rgb_img1, mask1, rgb_img2, mask2, sift_dist_th, ratio_th);
[A, B] = get_depth(pc1, pc2, sift_pairs);
%[model, pt_idx] = ransac(A, B, ransac_param);
[model, pt_idx] = ransac_icp(A, B, ransac_param);

%%
xyz_pc1 = pc1.Location;
color_pc1 = pc1.Color;
z_pc1 = xyz_pc1(:,3);
idx1 = find(z_pc1 > 4);
xyz_pc1(idx1,:) = 0;
color_pc1(idx1,:) = 0;
xyz_pc1 = cat(2, xyz_pc1, ones(size(xyz_pc1, 1), 1));
xyz_pc1_t = (model*xyz_pc1')';
xyz_pc1_t = xyz_pc1_t(:,1:3);
transformed_pc = pointCloud(xyz_pc1_t, 'Color', color_pc1);

figure(1);
pcshow(transformed_pc);

xyz_pc2 = pc2.Location;
color_pc2 = pc2.Color;
z_pc2 = xyz_pc2(:,3);
idx2 = find(z_pc2 > 4);
xyz_pc2(idx2,:) = 0;
color_pc2(idx2,:) = 0;
new_pc2 = pointCloud(xyz_pc2, 'Color', color_pc2);

figure(2);
pcshow(new_pc2);