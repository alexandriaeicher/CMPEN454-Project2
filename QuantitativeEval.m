list = zeros(12,size(new_joints,1));
metrics = zeros(12, 5);
for a = 1:12
    for z = 1:size(new_joints,1)
        list(a,z) = euclidean_distance(new_joints(z,a,1:3), C(1:3,z,a));
    end
    % Calculate metrics
    metrics(a,1) = mean(list(a,:));
    metrics(a,2) = std(list(a,:));
    metrics(a,3) = min(list(a,:));
    metrics(a,4) = median(list(a,:));
    metrics(a,5) = max(list(a,:));
    
    
end
% Calculate metrics for all joints
all_joints_metrics = zeros(1, 5);
all_joints_metrics(1, 1) = mean2(list);
all_joints_metrics(1, 2) = std2(list);
all_joints_metrics(1, 3) = min(list(:));
all_joints_metrics(1, 4) = median(list(:));
all_joints_metrics(1, 5) = max(list(:));
    

% Calculate error sums per frame
frame_range = 1:size(new_joints);
frame_error_sums = zeros(1,size(new_joints,1));
for z = 1:size(new_joints,1)
    frame_error_sums(1, z) = sum(list(:,z));
end

