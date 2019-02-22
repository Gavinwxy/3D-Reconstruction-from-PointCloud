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
    fprintf('Total pairs not enough !\n')
    return
end

inl_th = round(match_num*inl_ratio);
models = cell(1, itr_num);
squared_errors = Inf(1, itr_num);

for i=1:1:itr_num
    sample_idx = rand_idx(match_num, sample_size); % return random index
    sample_A = A(:,sample_idx);
    sample_B = B(:,sample_idx);
    
    remove_idx = [];
    visited_ax = [];
    
    sample_Ax = sample_A(1,:);
    for i = 1:sample_size
        ax = sample_Ax(i);
        idx = find(sample_Ax==ax);
        if (ismember(ax, visited_ax) == 0) && (length(idx) > 1)
            idx = idx(2:end);
            remove_idx = [remove_idx, idx];
            visited_ax = [visited_ax, ax];
        end
    end
    
    sample_A(:, remove_idx) = [];
    sample_B(:, remove_idx) = [];
    
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