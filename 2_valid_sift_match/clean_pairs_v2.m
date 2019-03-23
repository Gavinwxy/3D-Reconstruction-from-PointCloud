function sift_idx = second_best_check(sift_pairs, dist_collection, ratio_th)
% Check the ratio between distances of best match and second best match
% Args: 
%   sift_pairs: Index of SIFT pairs in point cloud. [x1, y1; x2, y2]
%   dist_collection: Corresponding distances
%   ratio_th: Threshold ratio to judge if it is a valid match
% Returns:
%   sift_idx: Filtered SIFT pairs
%
idx_left = sift_pairs(:,1);
idx_right = sift_pairs(:,2);

idx_no_rep_left = unique(idx_left, 'stable');
num_idx_left = length(idx_no_rep_left);

left_idx_temp = [];
right_idx_temp = [];

dist_temp = [];
sift_idx = [];

for i=1:1:num_idx_left
    target_idx = idx_no_rep_left(i);
    target_loc = find(idx_left==target_idx);
    if length(target_loc)<2
        left_idx_temp = [left_idx_temp; target_idx];
        right_idx_temp = [right_idx_temp; idx_right(target_loc)];
        dist_temp = [dist_temp; dist_collection(target_loc)];
        continue;
    end    
    left_extract = idx_left(target_loc);
    right_extract = idx_right(target_loc);
    dist_target = dist_collection(target_loc);
    
    [dist_sort, sort_idx] = sort(dist_target);
    left_sort = left_extract(sort_idx);
    right_sort = right_extract(sort_idx);
    
    ratio = dist_sort(1)/dist_sort(2);
    if ratio < ratio_th
        left_idx_temp = [left_idx_temp; left_sort(1)];
        right_idx_temp = [right_idx_temp; right_sort(1)];
        dist_temp = [dist_temp; dist_sort(1)];
    else
        continue;
    end
end

idx_no_rep_right = unique(right_idx_temp, 'stable');
num_idx_right = length(idx_no_rep_right);

for i=1:1:num_idx_right
    target_idx = idx_no_rep_right(i);
    target_loc = find(right_idx_temp==target_idx);   
    if length(target_loc)<2
        sift_idx = [sift_idx; left_idx_temp(target_loc) target_idx];
        continue;
    end
    left_extract = left_idx_temp(target_loc);
    right_extract = right_idx_temp(target_loc);
    dist_target = dist_temp(target_loc);
    [dist_sort, sort_idx] = sort(dist_target);
    left_sort = left_extract(sort_idx);
    right_sort = right_extract(sort_idx);
    ratio = dist_sort(1)/dist_sort(2);
    if ratio < ratio_th
        sift_idx = [sift_idx; left_sort(1) right_sort(1)];
    else
        continue;
    end
end

  