function sift_idx = second_best_check(sift_pairs, dist_collection, ratio_th)

idx_left = sift_pairs(:,1);
idx_right = sift_pairs(:,2);
idx_no_rep_left = unique(idx_left, 'stable');
num_idx = length(idx_no_rep_left);
left_idx_temp = [];
right_idx_temp = [];
dist_temp = [];
sift_idx = [];

for i=1:1:num_idx
    target_idx = idx_no_rep_left(i);
    target_loc = find(idx_left==target_idx);
    if length(target_loc)<2
        continue;
    end
    left = idx_left(target_loc);
    right = idx_right(target_loc);
    dist_target = dist_collection(target_loc);
    [dist_sort, sort_idx] = sort(dist_target);
    left = left(sort_idx);
    right = right(sort_idx);
    ratio = dist_sort(1)/dist_sort(2);
    if ratio < ratio_th
        left_idx_temp = [left_idx_temp; left(1)];
        right_idx_temp = [right_idx_temp; right(1)];
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
        continue;
    end
    left = left_idx_temp(target_loc);
    right = right_idx_temp(target_loc);
    dist_target = dist_temp(target_loc);
    [dist_sort, sort_idx] = sort(dist_target);
    left = left(sort_idx);
    right = right(sort_idx);
    ratio = dist_sort(1)/dist_sort(2);
    if ratio < ratio_th
        sift_idx = [sift_idx; left(1) right(1)];
    else
        continue;
    end
end

  