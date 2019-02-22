function cleaned_pairs = clean_pairs(pairs, distances, best_2nd_ratio)
    % left cleaning:
    left = pairs(:,1);
    uniq = unique(left);

    idx_cumulative = [];

    for i = 1:size(uniq, 1)
        x = uniq(i);

        idx = find(left==x);
        if length(idx) == 1
            continue;
        end

        dist_dup = distances(idx);

        [dist_smallest, idx_smallest] = min(dist_dup);
        dist_dup(idx_smallest) = Inf;
        dist_2nd_smallest = min(dist_dup);

        if dist_smallest / dist_2nd_smallest < best_2nd_ratio
            idx(idx_smallest) = [];
        end

        idx_cumulative = unique([idx_cumulative; idx]);
    end

    pairs(idx_cumulative,:) = [];
    distances(idx_cumulative,:) = [];
    
    % right cleaning:
    right = pairs(:,2);
    uniq = unique(right);

    idx_cumulative = [];

    for i = 1:size(uniq, 1)
        x = uniq(i);

        idx = find(right==x);
        if length(idx) == 1
            continue;
        end

        dist_dup = distances(idx);

        [dist_smallest, idx_smallest] = min(dist_dup);
        dist_dup(idx_smallest) = Inf;
        dist_2nd_smallest = min(dist_dup);

        if dist_smallest / dist_2nd_smallest < best_2nd_ratio
            idx(idx_smallest) = [];
        end

        idx_cumulative = unique([idx_cumulative; idx]);
    end

    pairs(idx_cumulative,:) = [];
    distances(idx_cumulative,:) = [];
    
    cleaned_pairs = pairs;
end