function pc_indices = remove_flying_pixels(pc, distance, number, x_dim)
    pc_indices = [];
    neighbor = 5;
    for i = 1:length(pc)
        total_distances = [];
        if sum(isnan(pc(i,:)))==0
            for mmm = i-x_dim*neighbor : x_dim : i+x_dim*neighbor
                low = max(mmm-neighbor, 1);
                low = min(low, size(pc, 1));
                high = min(mmm+neighbor, size(pc, 1));
                high = max(high, 1);
                if low == high
                    continue
                end
                distances = sum((pc(i,:) - pc(low : high, :)).^2, 2).^0.5;
                total_distances = [total_distances ; distances];
            end
            if sum(total_distances < distance) < number
                pc_indices = [pc_indices; i];
            end
        end
    end
end