function [path,costValue] = MVONEW(map, start, goal, astar)
% GSO Find the shortest path from start to goal.
%   PATH = GSO(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = GSO(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = GSO(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end
path = [];
num_expanded = 0;


% clear all
% filename = 'C:\Users\PC\Desktop\MS\Spring 2015\Advanced Robotics\Project-1\Phase-3\studentcode\maps\map1.txt';
% xy_res = 0.1;
% z_res = 2;
% margin = 0.25;
% astar = true;
% map = load_map(filename,xy_res,z_res,margin);
% start = [0.0  -4.9 0.2];
% goal = [6.0  18 3.0];

xy_res = map(3,4);
z_res = map(4,4);
startcopy = start;
goalcopy = goal;
startcollisioncheck = collide(map,start);
goalcollisioncheck = collide(map,goal);
if startcollisioncheck == 1 || goalcollisioncheck == 1
    fprintf('Start or Goal within obstacle\n');
    path = [];
    return;
end

Boundaryinitial = map(1,4:6);
Boundaryfinal = map(2,4:6);
if (Boundaryfinal(3)-Boundaryinitial(3))<z_res
    z_res = Boundaryfinal(3)-Boundaryinitial(3);
end
AllPoints = mygrid(Boundaryinitial,Boundaryfinal,xy_res,z_res);
DummyX = (Boundaryinitial(:,1):xy_res:Boundaryfinal(:,1))';
DummyY = (Boundaryinitial(:,2):xy_res:Boundaryfinal(:,2))';
DummyZ = (Boundaryinitial(:,3):z_res:Boundaryfinal(:,3))';
CollisionTest = collide(map,AllPoints);

if mod(goal(:,1),xy_res)
    goal(:,1) = goal(:,1) - mod(goal(:,1),xy_res);
end
if mod(goal(:,2),xy_res)
%     goal(:,2) = goal(:,2) + (xy_res-mod((goal(:,2)-start(:,2)),xy_res));
    goal(:,2) = goal(:,2) - mod(goal(:,2),xy_res);
end
if mod(goal(:,3),z_res)
    goal(:,3) = goal(:,3) - mod(goal(:,3),z_res);
%     goal(:,3) = goal(:,3) - mod((goal(:,3)-start(:,3)),z_res);
end
if mod(start(:,1),xy_res)
    start(:,1) = start(:,1) - mod(start(:,1),xy_res);
end
if mod(start(:,2),xy_res)
    start(:,2) = start(:,2) - mod(start(:,2),xy_res);
end
if mod(start(:,3),z_res)
    start(:,3) = start(:,3) - mod(start(:,3),z_res);
%     start(:,3) = start(:,3) - mod((start(:,3)-start(:,3)),z_res);
end
% DistanceFromStart = sqrt(sum(bsxfun(@minus, AllPoints, start).^2, 2));
MaxNoofNodes = size(AllPoints,1);
GenerateNodes = (1:MaxNoofNodes)';
% StartNode = GenerateNodes(ismember(AllPoints,start,'rows'));
StartNode = GenerateNodes(sum(abs(AllPoints(:,:)-ones(size(AllPoints,1),1)*start),2)<eps);
% GoalNode = GenerateNodes(ismember(AllPoints,goal,'rows'));
GoalNode = GenerateNodes(sum(abs(AllPoints(:,:)-ones(size(AllPoints,1),1)*goal),2)<eps);


%First Task it to generate population of solutions
% no of universes
nUni = 25;
MaxIt =1;

WEP_Max=1;
WEP_Min=0.2;
p=6;
WEP_init=0.2;
TDR_init=1;
%RANGE
% range_init = 5.0;
% range_boundary = 50.2;

%LUCIFERIN
% luciferin_init = 25;
% luciferin_decay = 0.4;
% luciferin_enhancement = 0.6;

%Neighbors

%defines the exploitation accuracy

% Create Empty Universe Structure
empty_universe.Position=[];
empty_universe.WEP=[];
empty_universe.TDR=[];
empty_universe.NV = [];

% Initialize Global Best
GlobalBest.Cost=inf;
GlobalBest.Position = [];
lb=[0 0 0]
ub=[20 5 6]
cost_func=[]
% Create universes Matrix
universes=repmat(empty_universe,nUni,1);
nodes = 200;
i = 1;
while i<=nUni
    [paths,CurrentNode,NVNodes] = getPath(CollisionTest,StartNode,GoalNode,AllPoints(:,:),nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1));
    %fprintf('%d %d\n',curNodeIndex, nodes);
       if CurrentNode == GoalNode 
           universes(i).Position = AllPoints(paths,:);
       elseif CurrentNode ~= GoalNode && NVNodes == (nodes + 1)
               universes(i).Position = AllPoints(paths,:);
       elseif NVNodes < nodes
           continue;
       else 
           universes(i).Position = AllPoints(paths,:);
       end
       universes(i).Position(:,4) = paths(:,1);
       universes(i).TDR = TDR_init;
       universes(i).NV = NVNodes;
       universes(i).WEP = WEP_init;
       universes(i).Cost = size(paths,1) + 100*pdist2(AllPoints(CurrentNode,:),AllPoints(GoalNode,:));
       cost_func(i)=universes(i).Cost;
       %fprintf('%d %d %d %d %d %d %d \n',AllPoints(CurrentNode,:),AllPoints(GoalNode,:),pdist2(AllPoints(CurrentNode,:),AllPoints(GoalNode,:)));
       if universes(i).Cost < GlobalBest.Cost
           GlobalBest.Cost = universes(i).Cost; 
           GlobalBest.Position = universes(i).Position;
       end
       i = i + 1;
end
% Algorithm initialization
[sorted_cost_func,sorted_indexes]=sort(cost_func);

for newindex=1:nUni
        Sorted_universes(newindex,:)=universes(sorted_indexes(newindex),:);
end

normalized_sorted_cost_func=normr(sorted_cost_func); 
Universes(1,:)= Sorted_universes(1,:);
for it=1:MaxIt
        WEP_C = WEP_Min+it*((WEP_Max-WEP_Min)/MaxIt);
        TDR_C = 1-((it)^(1/p)/(MaxIt)^(1/p));
    for i=2:nUni
        
        % Update WEP
        universes(i).WEP=WEP_Min+it*((WEP_Max-WEP_Min)/MaxIt);
        % Update TDR
        universes(i).TDR=1-((it)^(1/p)/(MaxIt)^(1/p));
        % assuming blackhole
        Back_hole_index=i;
        
            r1=rand();
            if r1<normalized_sorted_cost_func(i)
                White_hole_index=RouletteWheelSelection(-normalized_sorted_cost_func);% for maximization problem -sorted_Inflation_rates should be written as sorted_Inflation_rates
                if White_hole_index==-1
                    White_hole_index=1;
                end
                %Eq. (3.1) in the paper
                Universes(Back_hole_index).Position=Sorted_universes(White_hole_index).Position;
            end
            
                r2=rand();
                if r2<WEP_C
                    current = universes(Back_hole_index).Position;
                    toward = GlobalBest.Position;
            
            
                    [newPath,Cost] = changePath(current(:,4),toward(:,4),AllPoints(:,:),CollisionTest,StartNode,GoalNode,nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1));
                    newPath = removeLoops(newPath);
                    universes(Back_hole_index).Position = AllPoints(newPath(:,1),:);
                    universes(Back_hole_index).Position(:,4) = newPath(:,1);
                    universes(Back_hole_index).Cost = Cost;
%                     r3=rand();
%                     if r3<0.5
%                         Universes(i).Position=GlobalBest.Position+Universes(i).TDR*((ub-lb)*rand+lb);
%                     end
%                     if r3>0.5
%                         Universes(i,j)=Best_universe(1,j)-TDR*((ub-lb)*rand+lb);
%                     end

                elseif r2>=WEP_C
                    [newPath,Cost] = changePath(universes(Back_hole_index).Position(:,4),[],AllPoints(:,:),CollisionTest,StartNode,GoalNode,nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1));
                    newPath = removeLoops(newPath);  
                    universes(Back_hole_index).Position = AllPoints(newPath(:,1),:);
                    universes(Back_hole_index).Position(:,4) = newPath(:,1);
                    universes(Back_hole_index).Cost = Cost;
                end
            
        if  universes(Back_hole_index).Cost < GlobalBest.Cost
            GlobalBest.Cost = universes(Back_hole_index).Cost; 
            GlobalBest.Position = universes(Back_hole_index).Position;
        end
        
    end
end
        
path = GlobalBest.Position(:,1:3);
costValue = GlobalBest.Cost;
%fprintf('%d %d %d',size(path));
plot_path(map,path);
% plot3(path(:,1), path(:,2), path(:,3),'b');
% hold on;
% grid on;    
end

