
close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
v1=cputime;
map = load_map('maps/map3.txt', 0.1, 0.5, 0.25);
%%map = load_map('maps/map1.txt', 0.1, 0.5, 0.25);
%%map = load_map('maps/map2.txt', 0.1, 0.5, 0.25);
 start = {[14.1,2.8,5.8],[11.6,2,5],[8.1,3,5],[14,4,5],[11.2,4.5,5]};
 stop  = {[19.8,2,5],[19.8,4.5,5.6],[1.8,4,5.6],[2,3,5.9],[5,4,5.2]};

dist  = [0 0 0 0 0 ;0 0 0 0 0 ;0 0 0 0 0 ;0 0 0 0 0;0 0 0 0 0 ];
for i= 1:length(start)
    for j= 1:length(start)
        p1=start{i};
        p2=stop{j};
        disp(p1);
        dist_x = (p1(1,1)- p2(1,1))*(p1(1,1) - p2(1,1));
        dist_y = (p1(1,2)- p2(1,2))*(p1(1,2) - p2(1,2));
        dist_z = (p1(1,3)- p2(1,3))*(p1(1,3) - p2(1,3));
        dist(i,j) = sqrt(dist_x + dist_y + dist_z);
    end
end

[ass,cost]=munkres(dist);


start_temp = {};
stop_temp = {};
for i = 1:length(start)
    for j=1:length(start)
        if ass(i,j)==1
            start_temp{end+1}= start{i};
            stop_temp{end+1}= stop{j};
        end
    end
end
fprintf('Munkres Execution time = %d \n',cputime-v1);
start = start_temp;
stop= stop_temp;
disp(start)
disp(stop)
disp(length(start))
disp(length(stop))
sumall =0;
maxall = -1;
minall = 1000000000000;
sumallarray = [];
for sim = 1:10
   fprintf('Simulation number= %d \n',sim);  
%visited_nodes = [];
nquad = length(start);
costValue=0;
sumValue=0;
for qn = 1:nquad
    v = cputime;
    [path{qn},costValue] = MVONEW(map, start{qn}, stop{qn},true);
    c = cputime - v;
    
    sumValue = sumValue+costValue
    fprintf('Algo Execution time for quad %d = %d \n',qn,c);
end
 
 sumall = sumall+sumValue;
 sumallarray(end+1) = sumValue;
 if(sumValue>maxall)
     maxall=sumValue;
 end
 if(sumValue<minall)
     minall=sumValue;
 end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
    for qn = 1:nquad
         plot_path(map, path{qn});
    end
end
end
fprintf('Sum is %d and average is %d \n',sumall,sumall/10);
fprintf('Min is %d and Max is %d \n',minall,maxall);
fprintf('Standard deviation is %d \n',std(sumallarray));

%% Additional init script
init_script;