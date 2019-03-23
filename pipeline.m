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

%% Load trainig data
office = load('dataset/office1.mat');
pcs = office.pcl_train;
pcs(20:end) = [];

%% Preprocessing data

X = 640;
Y = 480;

% Exclude boundary points in image:
idx_upper = [];
idx_lower = [];

for i = 1:Y
    idx_upper = [idx_upper; 1 + (i-1)*X];
    idx_lower = [idx_lower; X + (i-1)*X];
end
idx_left = [2:X-1]';
idx_right = [X*Y-X+2:X*Y-1]';
idx = cat(1, idx_upper, idx_lower, idx_left, idx_right);

masks = {};

for i = 1:size(pcs, 2)
    xyzpc = pcs{i}.Location;
    xpc = xyzpc(:,1);
    ypc = xyzpc(:,2);
    zpc = xyzpc(:,3);
    
    % Exclude points with z > 4 in Location:
    idx1 = find(zpc > 4);
    
    % Exclude NaN points in Location:
    idx2 = find(isnan(xpc));
    idx3 = find(isnan(ypc));
    idx4 = find(isnan(zpc));
    
    %{
    % Remove flying points slowly but with high quality:
    radius = 0.3;
    min_neighbors = 6000;
    MdlKDT = KDTreeSearcher(xyzpc);
    
    idx5 = [];
    for j = 1:size(xyzpc, 1)
        Y = xyzpc(j,:);
        
        IdxKDT = rangesearch(MdlKDT,Y,radius);
        
        if size(IdxKDT{1}, 2) < min_neighbors
            idx5 = [idx5; j];
        end
    end
    %}
    
    % Remove flying points fastly:
    idx5 = remove_flying_pixels(xyzpc, 0.15, 100, X);

    idx_final = cat(1, idx, idx1, idx2, idx3, idx4, idx5);
    masks{i} = unique(idx_final);
end

%{
% Exclude Bob's body in Color:
idx_27 = masks{27};

pc = pcs{27};
colorpc = pc.Color;
colorpc(idx_27, :) = 0;
colorpc = reshape(colorpc, 640, 480, 3);

grayimg = rgb2gray(colorpc);
edcanny = edge(grayimg, 'Canny', 0.285);

SE = strel ('square', 4);
dilation = imdilate(edcanny, SE);

filledbob = imfill(dilation, 'holes');

SE2 = strel('square', 4);
body_mask=imerode(filledbob, SE2);

idx_body = find(body_mask == 1);

% Exclude Bob's legs in Color:
idx_2d = [];

start_x = 141;
start_y = 245;

width = 150;
height = 220;

for h = 0:220
    if mod(h, 5) == 0
        start_x = start_x - 1;
    end
    
    for w = 0:150
        idx_2d = [idx_2d; start_x + w, start_y + h];
    end
end

idx_legs = [];
for i = 1:size(idx_2d, 1)
    idx_legs = [idx_legs; idx_convert_2d_to_1d(idx_2d(i,:))];
end

% Exclude Bob as a whole from the scene:
idx_27 = cat(1, idx_27, idx_body, idx_legs);
masks{27} = unique(idx_27);
%}

% Save masks to mat file:
save('mask_collection.mat', 'masks');

%% Estimate transformations  
mask_collection = load('results/mask_collection.mat');
masks = mask_collection.masks;

min_frame = 1; % transform all frames after min_frame to min_frame's coordinate system
max_frame = size(pcs, 2); % the last frame to be transformed
models = cell(1,max_frame-1); % store up to 39 transformation matrices:

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
transformed_pcs = cell(1,max_frame); % store up to 40 transformed point clouds

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
max_frame = size(pcs, 2);
new_office = load('results/new_office.mat');
transformed_pcs = new_office.transformed_pcs;

% Merge point clouds from min_frame to max_frame
pc_merged = transformed_pcs{min_frame};
for frame = (min_frame+1):max_frame
    pc_merged = pcmerge(pc_merged, transformed_pcs{frame}, 0.015);
end

%% Evaluation
% angle, wall_dist = planefit(transformed_pcs);




