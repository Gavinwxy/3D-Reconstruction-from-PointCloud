rng(0);

addpath('./sift_match_transform');
pcs = load('dataset/office1.mat');
pcs = pcs.pcl_train;
mask_collection = load('mask_collection.mat');
masks = mask_collection.masks;

min_frame = 1;
max_frame = 2;
models = cell(1,max_frame-1);

for frame = max_frame:-1:(min_frame+1)
    frame1 = frame;
    frame2 = frame-1;
    
    pc1 = pcs{frame1};
    pc2 = pcs{frame2};
    
    mask1 = mask_convert_1d_to_2d(masks{frame1});
    mask2 = mask_convert_1d_to_2d(masks{frame2});
    
    rgb_img1 = imag2d(pc1.Color);
    rgb_img2 = imag2d(pc2.Color);
    
    % Hyper parameters
    sift_dist_th = 50; % the less the stricter
    area_ratio_th = 0.9; % the more the stricter
    best_2nd_ratio = 0.5; % the less the stricter
    ransac_param.sample_size = 20; % the more the stricter
    ransac_param.th_dist = 1; % the less the stricter
    ransac_param.itr_num = 100; % number of iteration
    ransac_param.inl_ratio = 0.5;% the more the stricter

    sift_pairs = valid_sift(rgb_img1, mask1, rgb_img2, mask2, sift_dist_th, area_ratio_th, best_2nd_ratio);
    [A, B] = get_depth(pc1, pc2, sift_pairs);
    [model, pt_idx] = ransac_icp(A, B, ransac_param);
    
    models{frame2} = model;
end

save('model_collection.mat', 'models');


model_collection = load('model_collection.mat');
models = model_collection.models;

transformed_pcs = cell(1,max_frame);


for frame = max_frame:-1:min_frame
    pc = pcs{frame};
    mask = masks{frame};
    
    color_pc = pc.Color;
    color_pc(mask,:) = [];
    
    xyz_pc = pc.Location;
    xyz_pc(mask,:) = [];
    xyz_pc = cat(2, xyz_pc, ones(size(xyz_pc, 1), 1));
    xyz_pc = xyz_pc';
    
    model_idx = frame - 1;
    while model_idx >= min_frame
        model = models{model_idx};
        xyz_pc = model*xyz_pc;
        model_idx = model_idx - 1;
    end
    
    xyz_pc = xyz_pc';
    xyz_pc = xyz_pc(:,1:3);
    pc_t = pointCloud(xyz_pc, 'Color', color_pc);
    transformed_pcs{frame} = pc_t;
end

save('new_office.mat', 'transformed_pcs');


new_office = load('new_office.mat');
transformed_pcs = new_office.transformed_pcs;

pc_merged = transformed_pcs{min_frame};
for frame = (min_frame+1):max_frame
    pc_merged = pcmerge(pc_merged, transformed_pcs{frame}, 0.015);
end

xyz_pc = pc_merged.Location;
color_pc = pc_merged.Color;
z_pc = xyz_pc(:,3);
idx = find(z_pc>4);
xyz_pc(idx,:) = 0;
pc_merged = pointCloud(xyz_pc, 'Color', color_pc);
