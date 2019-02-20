function [A, B] = get_depth(pc1, pc2, sift_pairs)
% Find (x,y,z) coord from index
A = [];
B = [];

depth1 = pc1.Location;
depth2 = pc2.Location;

num_pair = size(sift_pairs, 2);
for i = 1:1:num_pair
    loc1 = sift_pairs(1,i);
    loc2 = sift_pairs(2,i);
    A = [A, depth1(loc1, :)'];
    B = [B, depth2(loc2, :)'];
end

