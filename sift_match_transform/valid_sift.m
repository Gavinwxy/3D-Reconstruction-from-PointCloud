function sift_pairs = valid_sift(img1, mask1, img2, mask2, dist_th)
% This function is to filter and then find valid sift matches 
% Args:
%   img1, img2: RGB images
%   mask1, mask2: Masks indicating valid area for SIFT
%   dist_th: Distance threshold for sift matching.
% Returns:
%   sift_pairs: Matrix contatining location of sift pairs.
%               eg. [x1, x2, x3, x4]' (x1,x2) sift location in img1, 
%                   (x3, x4) sift location in img2.

img1_gray = single(rgb2gray(img1));
img2_gray = single(rgb2gray(img2));
[f1, d1] = vl_sift(img1_gray);
[f2, d2] = vl_sift(img2_gray);
pt_num1 = size(d1, 2);
pt_num2 = size(d2, 2);

% Filtered by mask
loc1 = round(f1(1:2,:));
loc2 = round(f2(1:2,:));

val_loc1 = [];
val_loc2 = [];
val_d1 = [];
val_d2 = [];

for i=1:1:pt_num1
    loc = loc1(:,i);
    if mask1(loc(1), loc(2)) == 1
        val_loc1 = [val_loc1, loc1(:,i)];
        val_d1 = [val_d1, d1(:,i)];
    end
end

for i=1:1:pt_num2
    loc = loc2(:,i);
    if mask2(loc(1), loc(2)) == 1
        val_loc2 = [val_loc2, loc2(:,1)];
        val_d2 = [val_d2, d2(:,1)];
    end
end

num_val_sift1 = size(val_loc1, 2);
num_val_sift2 = size(val_loc2, 2);

% Find matched pair
sift_pair = [];

for i=1:1:num_val_sift1
    for j=1:1:num_val_sift2
        dist = dist_cal(val_d1(:,i)-val_d2(:,j));
        if dist > dist_th
            continue;
        end
        sift_pairs = [sift_pair, val_loc1(:,i); val_loc2(:,j)];
    end
end
return

