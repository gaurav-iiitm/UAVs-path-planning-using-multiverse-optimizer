function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.
% clear all
% filename = 'C:\Users\PC\Desktop\MS\Spring 2015\Advanced Robotics\Project-1\Phase-3\studentcode\maps\map1.txt';
fileID = fopen(filename);
i = 1;
line = 'abc';
margin = margin;
% xy_res = 0.1;
% z_res = 2;
% margin = 0.25;
commentindex = 1;
boundaryindex = 1;
blockindex = 1;
k = 0;
while ~feof(fileID)
    line = fgetl(fileID);
    if isempty(line)
        continue;
    elseif line(1) == '#'
        Comments(commentindex,:) = textscan(line,'%s');
        commentindex = commentindex + 1;
    elseif line(2) == 'o'
        Boundary(boundaryindex,:) = textscan(line,'%s %f %f %f %f %f %f');
        boundaryindex = boundaryindex + 1;
    elseif line(2) == 'l'
        Block(blockindex,:) = textscan(line,'%s %f %f %f %f %f %f %f %f %f');
        blockindex = blockindex + 1;
        k = 1;
    end        
    i = i+1;
    
end
fclose(fileID);

x_0 = cell2mat(Boundary(1,2));
x_1 = cell2mat(Boundary(1,5));
y_0 = cell2mat(Boundary(1,3));
y_1 = cell2mat(Boundary(1,6));
z_0 = cell2mat(Boundary(1,4));
z_1 = cell2mat(Boundary(1,7));

     
j = 1;

if k==1
    for i=1:size(Block,1)

        xb_0 = cell2mat(Block(i,2));
        xb_1 = cell2mat(Block(i,5));
        yb_0 = cell2mat(Block(i,3));
        yb_1 = cell2mat(Block(i,6));
        zb_0 = cell2mat(Block(i,4));
        zb_1 = cell2mat(Block(i,7));

        xb_0 = xb_0-margin;
        xb_1 = xb_1+margin;
        yb_0 = yb_0-margin;
        yb_1 = yb_1+margin;
        zb_0 = zb_0-margin;
        zb_1 = zb_1+margin;
        
        if xb_0<x_0
            xb_0 = x_0;
        end
        if xb_1>x_1
            xb_1 = x_1;
        end
        if yb_0<y_0
            yb_0 = y_0;
        end
        if yb_1>y_1
            yb_1 = y_1;
        end
        if zb_0<z_0
            zb_0 = z_0;
        end
        if zb_1>z_1
            zb_1 = z_1;
        end


        B_1 = [xb_0 yb_0 zb_0]';
        B_2 = [xb_1 yb_0 zb_0]';
        B_3 = [xb_0 yb_0 zb_1]';
        B_4 = [xb_1 yb_0 zb_1]';
        B_5 = [xb_0 yb_1 zb_0]';
        B_6 = [xb_1 yb_1 zb_0]';
        B_7 = [xb_0 yb_1 zb_1]';
        B_8 = [xb_1 yb_1 zb_1]';


    %     BlockCoordinatesMatrix(j:j+7,:) = [B_1';B_2';B_3';B_4';B_5';B_6';B_7';B_8'];
        BlockCoordinatesMatrix(j:j+1,:) = [B_1';B_8'];
        BlockCoordinates(i,:) = {B_1 B_2 B_3 B_4 B_5 B_6 B_7 B_8};
        j = j+2;


        S_1 = [B_1 B_2 B_4 B_3];
        S_2 = [B_5 B_6 B_8 B_7];
        S_3 = [B_3 B_4 B_8 B_7];
        S_4 = [B_1 B_2 B_6 B_5];
        S_5 = [B_1 B_3 B_7 B_5];
        S_6 = [B_2 B_4 B_8 B_6];

        fill3([S_1(1,:)' S_2(1,:)' S_3(1,:)' S_4(1,:)' S_5(1,:)' S_6(1,:)'],[S_1(2,:)' S_2(2,:)' S_3(2,:)' S_4(2,:)' S_5(2,:)' S_6(2,:)'],[S_1(3,:)' S_2(3,:)' S_3(3,:)' S_4(3,:)' S_5(3,:)' S_6(3,:)'],[cell2mat(Block(i,8))/255 cell2mat(Block(i,9))/255 cell2mat(Block(i,10))/255]);
        xlabel('x'); ylabel('y'); zlabel('z'); 
        axis([min(x_0,x_1) (max(x_0,x_1)) min(y_0,y_1) (max(y_0,y_1)) min(z_0,z_1) (max(z_0,z_1))])
        grid
        hold on
    end

    map = BlockCoordinatesMatrix;
end
map(1:2,4:6) = [x_0 y_0 z_0;x_1, y_1, z_1];
map(3,4) = xy_res;
map(4,4) = z_res;

end

