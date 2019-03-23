office1 = load('office1.mat');
pcs = office1.pcl_train;

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
    
    % Remove flying points:
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
    
    % Remove flying points fastly:
    idx5 = remove_flying_pixels(xyzpc, 0.15, 100, X);

    idx_final = cat(1, idx, idx1, idx2, idx3, idx4, idx5);
    masks{i} = unique(idx_final);
end

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

% Save masks to mat file:
save('mask_collection.mat', 'masks');