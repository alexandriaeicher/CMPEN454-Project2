
%C are our the points we calculate with other functions 
%computes distance between the point(associated with joint)  and our
%point 
distances = zeros(12,size(new_joints,1));
for a = 1:12
    for z = 1:size(new_joints,1)
        distances(a,z) = euclidean_distance(new_joints(z,a,1:3), C(1:3,z,a));
        
        
    end
        
end
    
%euclidean distance function
function x = euclidean_distance(point1,point2)

x = sqrt(sum((point1(:) - point2(:)) .^ 2));

end