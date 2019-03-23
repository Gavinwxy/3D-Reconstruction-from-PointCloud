pcs = load('dataset/office1.mat');
pcs = pcs.pcl_train;
mask_collection1 = load('mask_collection.mat');
masks1 = mask_collection1.masks;

f = 11;

pc = pcs{f};
color = pc.Color;
loc = pc.Location;
figure(1);
imshow(imag2d(pc.Color));
%idx = find(loc(:,3)>4);
%loc(idx,:) = 0;
pc = pointCloud(loc, 'Color', color);
figure(2);
pcshow(pc);

pc = pcs{f};
mask1 = masks1{f};
color = pc.Color;
color(mask1,:) = 0;
loc = pc.Location;
loc(mask1,:) = 0;
pc = pointCloud(loc, 'Color', color);
figure(3);
pcshow(pc);