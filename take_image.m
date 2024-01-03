function img = take_image( pose, K, pattern_points )

img = K*[eye(3,3), zeros(3,1)]*pose*[pattern_points; ones(1,8)];

for i=1:8
    img(:,i) = img(:,i)/img(3,i);
end
