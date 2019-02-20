pc = load('dataset/office1.mat');
pc = pc.pcl_train;
pc1 = pc{8};
pc2 = pc{9};
a = load('mask_collection.mat');
masks = a.masks;

mask1 = mask_convert_1d_to_2d(masks{8});
mask2 = mask_convert_1d_to_2d(masks{9});

rgb_img1 = imag2d(pc1.Color);
rgb_img2 = imag2d(pc2.Color);

% Hyper parameters
sift_dist_th = 20;
ransac_param.sample_size = 10; % number of sample points to use
ransac_param.th_dist = 5; % distance threshold
ransac_param.itr_num = 100; % number of iteration
ransac_param.inl_ratio = 0.7;% inlier ratio

sift_pairs = valid_sift(rgb_img1, mask1, rgb_img2, mask2, sift_dist_th);
[A, B] = get_depth(pc1, pc2, sift_pairs);
[model, pt_idx] = ransac(A, B, ransac_param);

%%
xyz_pc1 = pc1.Location;
xyz_pc1 = cat(2, xyz_pc1, ones(size(xyz_pc1, 1), 1));
xyz_pc1_t = (model*xyz_pc1')';
xyz_pc1_t = xyz_pc1_t(:,1:3);

transformed_pc = pointCloud(xyz_pc1_t, 'Color', pc1.Color);
figure(1);
pcshow(transformed_pc);

xyz_pc2 = pc2.Location;
color_pc2 = pc2.Color;
zpc = xyz_pc2(:,3);
idx = find(zpc > 4);
xyz_pc2(idx,:) = 0;
color_pc2(idx,:) = 0;
new_pc2 = pointCloud(xyz_pc2, 'Color', color_pc2);
figure(2);
pcshow(new_pc2);