function [path,visited_nodes,costValue] = MVO(map, start, goal,visited_nodes)

persistent qd;

if qd 
    qd = qd+1;
else 
    qd =1;
end
xy_res = map(3,4);
z_res = map(4,4);

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

nPop = 25; %no. of universes
MaxIt = 50;

% Create Empty Particle Structure
empty_particle.Position=[];
%empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.NV = [];
empty_particle.w=[];

% Initialize Global Best

Dest.Position=[];
Dest.Cost=inf;

%leader.Cost=inf;
%leader.Position = [];


WEP_Max=1;
WEP_Min=0.2;

% beta.Cost=inf;
% beta.Position = [];
% 
% delta.Cost=inf;
% delta.Position = [];

% Create Particles Matrix
agent=repmat(empty_particle,nPop,1);
nodes = 100;
Time = 1;
i = 1;
while Time<MaxIt+1
    Time=Time+1;
    %Eq. (3.3) in the paper
    WEP=WEP_Min+Time*((WEP_Max-WEP_Min)/MaxIt);
    
    %Travelling Distance Rate (Formula): Eq. (3.4) in the paper
    TDR=1-((Time)^(1/6)/(MaxIt)^(1/6));
    
    %Inflation rates (I) (fitness values)
    %agent(i).cost=[];

while i<=nPop
    [paths,CurrentNode,NVNodes] = getPath(CollisionTest,StartNode,GoalNode,AllPoints(:,:),nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1),visited_nodes);
    %fprintf('%d %d\n',curNodeIndex, nodes);
       if CurrentNode == GoalNode 
           agent(i).Position = AllPoints(paths,:);
       elseif CurrentNode ~= GoalNode & NVNodes == (nodes + 1)
               agent(i).Position = AllPoints(paths,:);
       elseif NVNodes < nodes
           continue;
       else 
           agent(i).Position = AllPoints(paths,:);
       end
       agent(i).Position(:,4) = paths(:,1);
       agent(i).Velocity = 0;
       agent(i).NV = NVNodes;
       agent(i).Cost = size(paths,1) + 100*pdist2(AllPoints(CurrentNode,:),AllPoints(GoalNode,:));
       %fprintf('%d %d %d %d %d %d %d \n',AllPoints(CurrentNode,:),AllPoints(GoalNode,:),pdist2(AllPoints(CurrentNode,:),AllPoints(GoalNode,:)));
       
         if agent(i).Cost < Dest.Cost
           Dest.Position = agent(i).Position;
           Dest.Cost = agent(i).Cost; 
         end
        
       i = i + 1;
end
for i=1:nPop
[sorted_Inflation_rates,sorted_indexes]=sort([agent.Cost]);
end

    
    for newindex=1:nPop
        sorted(newindex).universes=agent(sorted_indexes(newindex)).Position;
    end
    
    %Normaized inflation rates (NI in Eq. (3.1) in the paper)
    normalized_sorted_Inflation_rates=normr(sorted_Inflation_rates);
    
    for i=1:nPop
    agent(i).Position= sorted(i).universes;
    end



 for it=2:MaxIt
    Back_hole_index = 1;
         for j=1:nPop
             
            for k=1:3 
             
                 r1=rand();
            if r1<normalized_sorted_Inflation_rates(j)
                White_hole_index=RouletteWheelSelection(-sorted_Inflation_rates);% for maximization problem -sorted_Inflation_rates should be written as sorted_Inflation_rates
                if White_hole_index==-1
                    White_hole_index=1;
                end
                %Eq. (3.1) in the paper
                agent(Back_hole_index).Position(k)=sorted(White_hole_index).universes(k);
            end
            
            lb= [0 0 0];
            ub=[20 5 6];
            
          
            if (size(lb,3)==1)
                
                %Eq. (3.2) in the paper if the boundaries are all the same
                r2=rand();
                if r2<WEP
                    r3=rand();
                    if r3<0.5
                        agent(j).Position(k)=Dest.Position(k)+TDR*((ub(k)-lb(k))*rand()+lb(k));
                      
                    end
                    if r3>0.5
                        agent(j).Position(k)=Dest.Position(k)-TDR*((ub(k)-lb(k))*rand()+lb(k));
                        
                    end
                end
                
            end
            
            if (size(lb,3)~=1)
              
                %Eq. (3.2) in the paper if the upper and lower bounds are
                %different for each variables
                r2=rand();
                if r2<WEP
                    r3=rand();
                    if r3<0.5
                        agent(j).Position(k)=Dest.Position(k)+TDR*((ub(k)-lb(k))*rand()+lb(k));
                    end
                    if r3>0.5
                        agent(j).Position(k)=Dest.Position(k)-TDR*((ub(k)-lb(k))*rand()+lb(k));
                    end
                end
            end
                
%             [newPath,Cost] = changePath(agent(j).Position(:,4),[],AllPoints(:,:),CollisionTest,StartNode,GoalNode,nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1),visited_nodes);
%             newPath = removeLoops(newPath);
%             agent(j).Position = AllPoints(newPath(:,1),:);
%             agent(j).Position(:,4) = newPath(:,1);
%             agent(j).Cost = Cost;
        end
       end
end

path = Dest.Position(:,1:3);

if path(end,:) ~= AllPoints(GoalNode,:)
    fprintf('Path for vehicle %d not reached goal\n',qd);
end
visited_nodes = [visited_nodes;Dest.Position(:,4)];
visited_nodes = unique(visited_nodes);

fprintf('Cost Function: %d  \n', Dest.Cost);
costValue = Dest.Cost
%fprintf('%d %d %d',size(path));
plot_path(map,path);
% plot3(path(:,1), path(:,2), path(:,3),'b');
% hold on;
% grid on;    
end

