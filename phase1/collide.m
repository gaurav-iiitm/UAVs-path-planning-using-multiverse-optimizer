function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
NoofBlocks = size(map,1)/2;
if ~map(end-1:end,1:3)
    NoofBlocks = NoofBlocks - 1;
end
if ~map(:,1:3)
    NoofBlocks = NoofBlocks - 2;
    C = zeros(size(points,1),1);
end
% if size(map,1)<=2
for i=1:size(points,1)
    for j=1:2:NoofBlocks*2
        if (points(i,1)>=map(j,1)&&points(i,2)>=map(j,2)&&points(i,3)>=map(j,3))
            if (points(i,1)<=map(j+1,1)&&points(i,2)<=map(j+1,2)&&points(i,3)<=map(j+1,3))
                C(i,1) = 1;
                break;
            else 
                C(i,1) = 0;
            end
        else
            C(i,1) = 0;
        end      
    end
end
end
