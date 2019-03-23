%% Load trainig data
office = load('results/office1.mat');
pcs = office.pcl_train;

%% Preprocessing data


%% Estimate transformations  
mask_collection = load('results/mask_collection.mat');
masks = mask_collection.masks;

min_frame = 1; % transform all frames after min_frame to min_frame's coordinate system
max_frame = 40; % the last frame to be transformed
% store up to 39 transformation matrices:
models = cell(1,max_frame-1);

% Setting Hyper parameters 
sift_dist_th = 60; % the less the stricter
area_ratio_th = 0.9; % the more the stricter
best_2nd_ratio = 0.7; % the less the stricter
ransac_param.sample_size = 20; % the more the stricter
ransac_param.th_dist = 0.1; % the less the stricter
ransac_param.itr_num = 100; % number of iteration
ransac_param.inl_ratio = 0.2; % the more the stricter
    
for frame = max_frame:-1:(min_frame+1)
    frame1 = frame;
    frame2 = frame-1;
    
    pc1 = pcs{frame1};
    pc2 = pcs{frame2};
    
    mask1 = mask_convert_1d_to_2d(masks{frame1});
    mask2 = mask_convert_1d_to_2d(masks{frame2});
    
    rgb_img1 = imag2d(pc1.Color);
    rgb_img2 = imag2d(pc2.Color);
    
    %Find valid sift match
    sift_pairs = valid_sift(rgb_img1, mask1, rgb_img2, mask2, sift_dist_th, area_ratio_th, best_2nd_ratio);
    [A, B] = get_depth(pc1, pc2, sift_pairs);
    [model, pt_idx] = ransac_icp(A, B, ransac_param);
    
    models{frame2} = model;
end
save('results/model_collection.mat', 'models');

%% Transform frames based on models
model_collection = load('results/model_collection.mat');
models = model_collection.models;
% store up to 40 transformed point clouds
transformed_pcs = cell(1,max_frame);

% Transform point clouds [min_frame+1, max_frame] toward min_frame
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
save('results/new_office.mat', 'transformed_pcs');

%% Fuse frames 
min_frame = 1;
max_frame = 40;
new_office = load('results/new_office.mat');
transformed_pcs = new_office.transformed_pcs;

% Merge point clouds from min_frame to max_frame
pc_merged = transformed_pcs{min_frame};
for frame = (min_frame+1):max_frame
    pc_merged = pcmerge(pc_merged, transformed_pcs{frame}, 0.015);
end

%% Evaluation
% angle, wall_dist = planefit(transformed_pcs);




