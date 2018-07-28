function [collide,collision_pairs] = collision_check(p, margin)
collide = 0;
if(size(p,1) <= 1)
    return;
end
p(:,3) = p(:,3)/3; % scale z-axis by 3 to make it ellipsoid
count = 1;
collision_pairs = [];

for y = 1:size(p,1)
    for z = y+1:size(p,1)
        dis = pdist2(p(y,:),p(z,:));
        if dis == 0 
            collide = 2;
            collision_pairs.x(count) = y;
            collision_pairs.y(count) = z;
            return;
        elseif dis < 2*margin
            collide = 1;
            collision_pairs.x(count) = y;
            collision_pairs.y(count) = z;
            count = count+1;
        end
    end
end

