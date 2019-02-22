function sift_pairs = valid_sift(img1, mask1, img2, mask2, dist_th, area_ratio_th, best_2nd_ratio)
% This function is to filter and then find valid sift matches 
% Args:
%   img1, img2: RGB images
%   mask1, mask2: Masks indicating valid area for SIFT
%   dist_th: Distance threshold for sift matching.
% Returns:
%   sift_pairs: Matrix contatining index of sift pairs (in depth image).

img1_gray = single(rgb2gray(img1));
img2_gray = single(rgb2gray(img2));
[f1, d1] = vl_sift(img1_gray);
[f2, d2] = vl_sift(img2_gray);
pt_num1 = size(d1, 2);
pt_num2 = size(d2, 2);

% Filtered by mask
loc1 = floor(f1(1:2,:));
loc2 = floor(f2(1:2,:));

val_loc1 = [];
val_loc2 = [];
val_d1 = [];
val_d2 = [];

for i=1:1:pt_num1
    loc = loc1(:,i);
    x_min = max(loc(1)-8, 1);
    x_max = min(loc(1)+8, 640);
    y_min = max(loc(2)-8, 1);
    y_max = min(loc(2)+8, 480);
    patch = mask1(x_min:x_max, y_min:y_max);
    [h, w] = size(patch);
    ratio = sum(patch, 'all') / (h * w);
    
    if ratio >= area_ratio_th
        if mask1(loc(1), loc(2)) == 1
            val_loc1 = [val_loc1, loc];
            val_d1 = [val_d1, d1(:,i)];
        elseif mask1(loc(1)+1, loc(2)) == 1
            loc(1) = loc(1) + 1;
            val_loc1 = [val_loc1, loc];
            val_d1 = [val_d1, d1(:,i)];
        elseif mask1(loc(1), loc(2)+1) == 1
            loc(2) = loc(2) + 1;
            val_loc1 = [val_loc1, loc];
            val_d1 = [val_d1, d1(:,i)];
        elseif mask1(loc(1)+1, loc(2)+1) == 1
            loc(1) = loc(1) + 1;
            loc(2) = loc(2) + 1;
            val_loc1 = [val_loc1, loc];
            val_d1 = [val_d1, d1(:,i)];
        end
    end
end

for i=1:1:pt_num2
    loc = loc2(:,i);
    x_min = max(loc(1)-8, 1);
    x_max = min(loc(1)+8, 640);
    y_min = max(loc(2)-8, 1);
    y_max = min(loc(2)+8, 480);
    patch = mask2(x_min:x_max, y_min:y_max);
    ratio = sum(patch, 'all') / area_ratio_th;
    
    if ratio >= area_ratio_th
        if mask2(loc(1), loc(2)) == 1
            val_loc2 = [val_loc2, loc];
            val_d2 = [val_d2, d2(:,i)];
        elseif mask2(loc(1)+1, loc(2)) == 1
            loc(1) = loc(1) + 1;
            val_loc2 = [val_loc2, loc];
            val_d2 = [val_d2, d2(:,i)];
        elseif mask2(loc(1), loc(2)+1) == 1
            loc(2) = loc(2) + 1;
            val_loc2 = [val_loc2, loc];
            val_d2 = [val_d2, d2(:,i)];
        elseif mask2(loc(1)+1, loc(2)+1) == 1
            loc(1) = loc(1) + 1;
            loc(2) = loc(2) + 1;
            val_loc2 = [val_loc2, loc];
            val_d2 = [val_d2, d2(:,i)];
        end
    end
end

num_val_sift1 = size(val_loc1, 2);
num_val_sift2 = size(val_loc2, 2);

% Find matched pair
sift_pairs = [];
distances = [];

for i=1:1:num_val_sift1
    for j=1:1:num_val_sift2
        dist = dist_chi_sq(val_d1(:,i), val_d2(:,j));
        if dist < dist_th
            % Convert pixel location to depth coord
            idx1 = idx_convert_2d_to_1d(val_loc1(:,i));
            idx2 = idx_convert_2d_to_1d(val_loc2(:,j));
            sift_pairs = [sift_pairs; idx1,idx2];
            distances = [distances; dist];
        end
    end
end

sift_pairs = clean_pairs(sift_pairs, distances, best_2nd_ratio);
sift_pairs = sift_pairs';

%{
sift_pairs = second_best_check(sift_pairs, distances', 1.5);

left_sift_pairs = sift_pairs(1,:);
right_sift_pairs = sift_pairs(2,:);
sift_pairs = [right_sift_pairs; left_sift_pairs];

sift_pairs = second_best_check(sift_pairs, distances', 1.5);
%}
return