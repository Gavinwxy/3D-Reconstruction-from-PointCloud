new_office = load('new_office.mat');
transformed_pcs = new_office.transformed_pcs;

% left wall with bookshelf
lw_frame_min = 1; 
lw_frame_max = 13;
% frontal wall with window
fw_frame_min = 14;
fw_frame_max = 24;
% right wall with white board
rw_frame_min = 25;
rf_frame_max = 26;

wall_sec = [lw_frame_min fw_frame_min rw_frame_min; 
            lw_frame_max fw_frame_max rf_frame_max];
        
% result of plane fitting. 
% First row contains point cloud of wall
% Second row contains normal vector
wall_result = cell(2,3);
%%
dist_th = 0.04;
for i=1:3
    min_frame = wall_sec(1,i);
    max_frame = wall_sec(2,i);
    pc_merged = transformed_pcs{min_frame};
    for frame = (min_frame+1):max_frame
        pc_merged = pcmerge(pc_merged, transformed_pcs{frame}, 0.015);
    end
    [model, inlier, ~] = pcfitplane(pc_merged, dist_th);
    plane = select(pc_merged, inlier);
    wall_result{1,i} = plane;
    wall_result{2,i} = model.Normal;
end
%%
vec_angle = @(x,y) atan2(norm(cross(x,y)), dot(x,y));
angle_mat = zeros(1,3);
angle_mat(1,1) = vec_angle(wall_result{2,1}, wall_result{2,2});
angle_mat(1,2) = vec_angle(wall_result{2,1}, wall_result{2,3});
angle_mat(1,3) = vec_angle(wall_result{2,2}, wall_result{2,3});
