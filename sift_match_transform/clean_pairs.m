function cleaned_pairs = clean_pairs(pairs, distances, best_2nd_ratio)
% Check the ratio between distances of best match and second best match
% Args: 
%   pairs: Index of SIFT pairs in point cloud. [x1, y1; x2, y2; ...]
%   distances: Corresponding distances
%   best_2nd_ratio: Threshold ratio to judge if it is a valid match
% Returns:
%   cleaned_pairs: Filtered SIFT pairs

    % left cleaning:
    left = pairs(:,1);
    uniq = unique(left);

    % pair indexes to be eliminated
    idx_cumulative = [];

    for i = 1:size(uniq, 1)
        x = uniq(i);

        idx = find(left==x);
        
        % if this point on the left involves only one pair, then keep it
        % and continue
        if length(idx) == 1
            continue;
        end

        dist_dup = distances(idx);

        [dist_smallest, idx_smallest] = min(dist_dup);
        dist_dup(idx_smallest) = Inf;
        dist_2nd_smallest = min(dist_dup);

        % If the best pair passes ratio test, then keep it and
        % discard the rest. Otherwise elminate all of them.
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
        
        % if this point on the right involves only one pair, then keep it
        % and continue
        if length(idx) == 1
            continue;
        end

        dist_dup = distances(idx);

        [dist_smallest, idx_smallest] = min(dist_dup);
        dist_dup(idx_smallest) = Inf;
        dist_2nd_smallest = min(dist_dup);

        % If the best pair passes ratio test, then keep it and
        % discard the rest. Otherwise elminate all of them.
        if dist_smallest / dist_2nd_smallest < best_2nd_ratio
            idx(idx_smallest) = [];
        end

        idx_cumulative = unique([idx_cumulative; idx]);
    end

    pairs(idx_cumulative,:) = [];
    distances(idx_cumulative,:) = [];
    
    cleaned_pairs = pairs;
end