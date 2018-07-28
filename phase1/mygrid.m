function [Mesh] = mygrid(start, goal,xy_res,z_res)
% start = currentpoint;
% goal = newpoint;
% xy_res = 0.1;
% z_res = 2;
if z_res==0
    x = start(:,1):xy_res:goal(:,1);
    y = start(:,2):xy_res:goal(:,2);
    [x, y, z] = meshgrid(x, y, start(:,3));
    Mesh = [x(:), y(:), z(:)];
    return;
end
x = min(start(:,1),goal(:,1)):xy_res:max(start(:,1),goal(:,1));
y = min(start(:,2),goal(:,2)):xy_res:max(start(:,2),goal(:,2));
z = min(start(:,3),goal(:,3)):z_res:max(start(:,3),goal(:,3));
[x, y, z] = meshgrid(x, y, z);
Mesh = [x(:), y(:), z(:)];
% Mesh(:,4) = 1:size(Mesh,1);
end