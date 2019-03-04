new_office = load('new_office.mat');
transformed_pcs = new_office.transformed_pcs;

% left wall with bookshelf
lw_frame_min = 1; 
lw_frame_max = 13;
% frontal wall with window
fw_frame_min = 14;
fw_frame_max = 23;
% right wall with white board
rw_frame_min = 24;
rf_frame_max = 40;

wall_sec = [lw_frame_min fw_frame_min rw_frame_min; 
            lw_frame_max fw_frame_max rf_frame_max];
        
% result of plane fitting. 
% First row contains point cloud of wall
% Second row contains normal vector
wall_result = cell(2,3);
%%
dist_th = 0.07;
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
% Function to calculate angle between vectors
vec_angle = @(x,y) atan2(norm(cross(x,y)), dot(x,y)); 
angle_mat = zeros(1,3);
angle_mat(1,1) = vec_angle(wall_result{2,1}, wall_result{2,2}); % Angle between left and end walls
angle_mat(1,2) = vec_angle(wall_result{2,1}, wall_result{2,3}); % Angle between left and right walls
angle_mat(1,3) = vec_angle(wall_result{2,2}, wall_result{2,3}); % Angel between end and right walls

%%
pcshow(wall_result{1,1})
hold on
pcshow(wall_result{1,2})
hold on
pcshow(wall_result{1,3})
hold on

%% Calculate wall center of mass based on raw data
wall_left_loc = wall_result{1,1}.Location;
wall_right_loc = wall_result{1,3}.Location;

center_left = sum(wall_left_loc,1)/size(wall_left_loc,1);
center_right = sum(wall_right_loc,1)/size(wall_right_loc,1);
distance1 = pdist([center_left;center_right], 'Euclidean');
%% Reconstruct even plane by equations
normal_vec_left = wall_result{2,1};
normal_vec_right = wall_result{2,3};

min_x_left = min(wall_left_loc(:,1));
max_x_left = max(wall_left_loc(:,1));
min_y_left = min(wall_left_loc(:,2));
max_y_left = max(wall_left_loc(:,2));

min_x_right = min(wall_right_loc(:,1));
max_x_right = max(wall_right_loc(:,1));
min_y_right = min(wall_right_loc(:,2));
max_y_right = max(wall_right_loc(:,2));

%% Get plane parameters
D_left = -dot(wall_left_loc(1,:),normal_vec_left);
D_right = -dot(wall_right_loc(1,:),normal_vec_right);

A_left = normal_vec_left(1);
B_left = normal_vec_left(2);
C_left = normal_vec_left(3);

A_right = normal_vec_right(1);
B_right = normal_vec_right(2);
C_right = normal_vec_right(3);

x_interval_left = (max_x_left - min_x_left)/100;
y_interval_left = (max_y_left - min_y_left)/100;

x_interval_right = (max_x_right - min_x_right)/100;
y_interval_right = (max_y_right - min_y_right)/100;

%% Plot plane
[x1, y1] = meshgrid(min_x_left:x_interval_left:max_x_left, min_y_left:y_interval_left:max_y_left); % Generate x and y data
z1 = -1/C_left*(A_left*x1 + B_left*y1 + D_left); % Solve for z data
surf(x1,y1,z1) %Plot the surface
hold on 
[x2, y2] = meshgrid(min_x_right:x_interval_right:max_x_right, min_y_right:y_interval_right:max_y_right); % Generate x and y data
z2 = -1/C_right*(A_right*x2 + B_right*y2 + D_right); % Solve for z data
surf(x2,y2,z2) %Plot the surface

%% Calculate wall center of mass based on even plane
new_center_left = [sum(x1,'all'), sum(y1,'all'), sum(z1,'all')]/(size(x1,2)^2);
new_center_right = [sum(x2,'all'), sum(y2,'all'), sum(z2,'all')]/(size(x2,2)^2);
distance2 = pdist([new_center_left;new_center_right], 'Euclidean');
% Check wether the distance make sense.
error = abs(distance1 - distance2);