%% Load the training data 
office = load('office1.mat');
office = office.pcl_train;
%% Uncomment to load the test file
% office = load('office2.mat');
% office = office.pcl_test;
%%
for i = 1:length(office) % Reading the 40 point-clouds  
    rgb = office{i}.Color; % Extracting the colour data
    point = office{i}.Location; % Extracting the xyz data
    pc = pointCloud(point, 'Color', rgb); % Creating a point-cloud variable
    figure(1)
    pcshow(pc)
    figure(2)
    imag2d(rgb) % Shows the 2D images
    pause
end
