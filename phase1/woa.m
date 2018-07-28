function [path,visited_nodes] = woa(map, start, goal,visited_nodes)

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


nPop = 30;
MaxIt = 40;

% Create Empty Particle Structure
empty_particle.Position=[];
%empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.NV = [];
empty_particle.w=[];

% Initialize Global Best
leader.Cost=inf;
leader.Position = [];

% beta.Cost=inf;
% beta.Position = [];
% 
% delta.Cost=inf;
% delta.Position = [];

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
       
            
       if agent(i).Cost < leader.Cost
           leader.Cost = agent(i).Cost; 
           leader.Position = agent(i).Position;
       end
       
%        if      agent(i).Cost > alpha.Cost &&  agent(i).Cost < beta.Cost
%            beta.Cost = agent(i).Cost; 
%            beta.Position = agent(i).Position;
%        end
%        
%        if   agent(i).Cost > alpha.Cost &&  agent(i).Cost > beta.Cost && agent(i).Cost < delta.Cost
%            delta.Cost = agent(i).Cost; 
%            delta.Position = agent(i).Position;
%        end
       
       i = i + 1;
end
% Algorithm initialization
VarSize = nPop;
% for it=1:MaxIt
%         
%          a = 2-it*((2)/MaxIt);    % a decreases linearly from 2 to 0
% %          a = 2*(1-((it)^2/(MaxIt)^2));
%         a2 = -1+t*((-1)/MaxIt);
%         
%         for j=1:nPop
%             for k=1:3 
%             
%                 r1 = rand();
%                 r2 = rand();
%                 
%                 A=2*a*r1-a;
%                 C=2*r2;
%                           
%                 b=1;
%                 l=(a2-1)*rand+1;
%                 
%                 p=rand();
%                 
%                 if p<0.5
%                     if abs(A)>=1
%                         rand_leader_index = floor(nPop*rand()+1);
%                         X_rand = agent(i).Position;
%                         D_X_rand = abs(C*X_rand(k)-agent(i).Position(k));
%                         agent(i).Position(k) = X_rand(k)-A*D_X_rand;
%                         
%                     elseif abs(A)<1
%                         D_leader = abs(C*leader.Position(k)-agent(i).Position(k));
%                         agent(i).Position(k) = leader.Position(k)-A*D_leader; 
%                     end
%                 elseif p>=0.5
%                     distance2Leader = abs(leader.Position(k)-agent(i).Position(k));
%                     agent(i).Position(k) = distance2Leader*exp(b.*l).*cos(l.*2*pi)+leader.Position(k);
%                 end
%             end
%         
%             [newPath,Cost] = changePath(agent(j).Position(:,4),[],AllPoints(:,:),CollisionTest,StartNode,GoalNode,nodes,Boundaryinitial,Boundaryfinal,size(DummyX,1),size(DummyY,1),size(DummyZ,1),visited_nodes);
%             newPath = removeLoops(newPath);
%             agent(j).Position = AllPoints(newPath(:,1),:);
%             agent(j).Position(:,4) = newPath(:,1);
%             agent(j).Cost = Cost;
%         end
%   
% end

path = leader.Position(:,1:3);

if path(end,:) ~= AllPoints(GoalNode,:)
    fprintf('Path for vehicle %d not reached goal\n',qd);
end
visited_nodes = [visited_nodes;leader.Position(:,4)];
visited_nodes = unique(visited_nodes);

fprintf('%d \n', leader.Cost);
%fprintf('%d %d %d',size(path));
plot_path(map,path);
% plot3(path(:,1), path(:,2), path(:,3),'b');
% hold on;
% grid on;    
end

