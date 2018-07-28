function [path,visited_nodes] = gwo(map, start, goal,visited_nodes)

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


nPop = 25;
MaxIt = 35;

% Create Empty Particle Structure
empty_particle.Position=[];
%empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.NV = [];
empty_particle.w=[];

% Initialize Global Best
alpha.Cost=inf;
alpha.Position = [];

beta.Cost=inf;
beta.Position = [];

delta.Cost=inf;
delta.Position = [];

% Create Particles Matrix
agent=repmat(empty_particle,nPop,1);
nodes = 100;
i = 1;
while i<=nPop
    [paths,CurrentNode,NVNodes] = getPath(CollisionTest,StartNode,GoalNode,AllPoints(:,:),nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1),visited_nodes);
    %fprintf('%d %d\n',curNodeIndex, nodes);
       if CurrentNode == GoalNode 
           agent(i).Position = AllPoints(paths,:);
       elseif CurrentNode ~= GoalNode && NVNodes == (nodes + 1)
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
       
            
       if agent(i).Cost < alpha.Cost
           alpha.Cost = agent(i).Cost; 
           alpha.Position = agent(i).Position;
       end
       
       if      agent(i).Cost > alpha.Cost &&  agent(i).Cost < beta.Cost
           beta.Cost = agent(i).Cost; 
           beta.Position = agent(i).Position;
       end
       
       if   agent(i).Cost > alpha.Cost &&  agent(i).Cost > beta.Cost && agent(i).Cost < delta.Cost
           delta.Cost = agent(i).Cost; 
           delta.Position = agent(i).Position;
       end
       
       i = i + 1;
end
% Algorithm initialization
VarSize = nPop;
for it=1:MaxIt
        
         a = 2-it*((2)/MaxIt);    % a decreases linearly from 2 to 0
%          a = 2*(1-((it)^2/(MaxIt)^2));
        for j=1:nPop
            for k=1:3 
            
                r1 = rand();
                r2 = rand();
                
                A1 = 2*a*r1-a;
                C1 = 2*r2;
                
                D_alpha = abs(C1*alpha.Position(k)-agent(j).Position(k));
                X1 = alpha.Position(k)-A1*D_alpha;
                
                r1 = rand();
                r2 = rand();
                
                A2 = 2*a*r1-a;
                C2 = 2*r2;
                
                D_beta = abs(C2*beta.Position(k)-agent(j).Position(k));
                X2 = beta.Position(k)-A2*D_beta;
                
                r1 = rand();
                r2 = rand();
                
                A3 = 2*a*r1-a;
                C3 = 2*r2;
                
                D_delta = abs(C3*delta.Position(k)-agent(j).Position(k));
                X3 = delta.Position(k)-A3*D_delta;
                
                agent(j).Position(k) = (X1+X2+X3)/3;
                
        
            end
            [newPath,Cost] = changePath(agent(j).Position(:,4),[],AllPoints(:,:),CollisionTest,StartNode,GoalNode,nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1),visited_nodes);
            newPath = removeLoops(newPath);
            agent(j).Position = AllPoints(newPath(:,1),:);
            agent(j).Position(:,4) = newPath(:,1);
            agent(j).Cost = Cost;
        end
        
        
        
        for i=1:nPop
            
        if agent(i).Cost < alpha.Cost
           alpha.Cost = agent(i).Cost; 
           alpha.Position = agent(i).Position;
       end
       
       if      agent(i).Cost > alpha.Cost &&  agent(i).Cost < beta.Cost
           beta.Cost = agent(i).Cost; 
           beta.Position = agent(i).Position;
       end
       
       if   agent(i).Cost > alpha.Cost &&  agent(i).Cost > beta.Cost && agent(i).Cost < delta.Cost
           delta.Cost = agent(i).Cost; 
           delta.Position = agent(i).Position;
       end
        
       end          
        
end
path = alpha.Position(:,1:3);

if path(end,:) ~= AllPoints(GoalNode,:)
    fprintf('Path for vehicle %d not reached goal\n',qd);
end
visited_nodes = [visited_nodes;alpha.Position(:,4)];
visited_nodes = unique(visited_nodes);

fprintf('%d \n', alpha.Cost);
%fprintf('%d %d %d',size(path));
plot_path(map,path);
% plot3(path(:,1), path(:,2), path(:,3),'b');
% hold on;
% grid on;    
end

