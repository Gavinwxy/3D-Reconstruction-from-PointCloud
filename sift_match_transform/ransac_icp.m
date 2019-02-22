function [model, pt_idx] = ransac(A, B, ransac_param)
% Args:
%   A, B: Point set A and B each with size m x n. m points with n
%         dimensions. [x, y, z]' for example.
%   ransac_param: Parameter setting for ransac algorithm 
% Returns:
%   model: Matrix containing Rotation and Translation matrix. [R, T; 0, 1]
%   pt_idx: vector of matched point indices 

sample_size = ransac_param.sample_size; % number of sample pairs to use
th_dist = ransac_param.th_dist; % distance threshold
itr_num = ransac_param.itr_num; % number of iteration
inl_ratio = ransac_param.inl_ratio;% inlier ratio

if sample_size < 3
    fprintf('Need more sample pairs to fit !\n');
    return
end

% transform to homogenous coord
match_num = size(A, 2);
A_homo = [A; ones(1, match_num)];
B_homo = [B; ones(1, match_num)];

if match_num < sample_size
    fprintf('Total pairs not enough in the first place !\n')
    return
end

inl_th = round(match_num*inl_ratio);
models = cell(1, itr_num);
squared_errors = Inf(1, itr_num);

for i=1:1:itr_num
    indexes = [1:match_num];
    
    idx = randsample(indexes, 1);
    indexes(find(indexes==idx)) = [];
    
    sample_A = A(:,idx);
    sample_B = B(:,idx);
    while size(sample_A, 2) < sample_size
        if size(indexes, 2) == 0
            fprintf('Total pairs not enough after removing repetitive pairs !\n')
            return
        end
        idx = randsample(indexes, 1);
        indexes(find(indexes==idx)) = [];
        
        a = A(:,idx);
        b = B(:,idx);
        sample_Ax = sample_A(1,:);
        if ismember(a(1), sample_Ax) == 0
            sample_A = [sample_A, a];
            sample_B = [sample_B, b];
        end
    end
    
    %%%%%%%%%%%%%%%%%% ICP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    pc_A = pointCloud(sample_A');
    pc_B = pointCloud(sample_B');
    F = pcregistericp(pc_B,pc_A,'Extrapolate',true);
    F = F.T;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    dist = dist_cal(F*A_homo, B_homo);
    inl_num = length(find(dist<th_dist));
    if inl_num < inl_th
        continue;
    end
    squared_errors(i) = sum(dist.^2);
    models{i} = F;
end

[err, idx] = min(squared_errors);
if err == Inf
    fprintf('Mo model meets success criteria !\n')
    return
end

model = models{idx};
dist = dist_cal(model*A_homo, B_homo);
pt_idx = find(dist < th_dist);

return