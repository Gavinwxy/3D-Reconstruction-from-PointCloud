function final_trans = transform_solve(pc_a, pc_b)
% This function is to calculate rotation and translation between
% two set of n dimesion points. 
%
% args:
%   pc_a: point set a with n points each of m dimensions
%   pc_b: point set b with the same size as a
% Return:
%   t_mat: optimal translation matrix
%   r_mat: optimal rotation matrix
% In this case, pc_b = pc_a * r_mat + t_mat

% Data centerlization
dim_mean_a = mean(pc_a, 2);
dim_mean_b = mean(pc_b, 2);
ct_a = pc_a - dim_mean_a;
ct_b = pc_b - dim_mean_b;

% Calculate correlation matrix
[dim_pc,num_pc] = size(pc_a);
col_mat = zeros(dim_pc, dim_pc);
for i = 1:1:num_pc
    tmp_mat = ct_a(:,i)*ct_b(:,i)';
    col_mat = col_mat+tmp_mat;
end

% Singular value decomposition

[U,~,V] = svd(col_mat);
R = V*U';
ct_a = mean(pc_a, 2);
ct_b = mean(pc_b, 2);
T = ct_b - R*ct_a;
zero_mat = zeros(1, size(R, 2));
final_trans = [R, T; zero_mat, 1];
return