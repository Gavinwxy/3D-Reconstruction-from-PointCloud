function sift_idx = second_best_check(sift_pairs, dist_collection, ratio_th)

idx_left = sift_pairs(1,:);
idx_right = sift_pairs(2,:);
idx_no_rep = unique(idx_left, 'stable');
num_idx = length(idx_no_rep);
sift_idx = [];

for i=1:1:num_idx
    target_idx = idx_no_rep(i);
    target_loc = find(idx_left==target_idx);
    left = idx_left(target_loc);
    right = idx_right(target_loc);
    dist_target = dist_collection(target_loc);
    [dist_sort, sort_idx] = sort(dist_target);
    left = left(sort_idx);
    right = right(sort_idx);
    ratio = dist_sort(2)/dist_sort(1);
    if ratio > ratio_th
        sift_idx = [sift_idx; left(1) right(1)];     
    else
        continue;
    end
end

  