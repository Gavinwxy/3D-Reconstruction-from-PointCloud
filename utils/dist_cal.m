function dist = dist_cal(x, y)
dist = sqrt(sum((x - y).^2, 1));
end