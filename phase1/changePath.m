function [ newpath,Cost ] = changePath( weakSol, strongSol, AllPoints,CollisionTest,~,GoalNode,nodes,Boundaryinitial,Boundaryfinal,s1,s2,s3,visited_nodes )
%CHANGEPATH Summary of this function goes here
%   Detailed explanation goes here
   if ~isempty(strongSol)
       C = intersect(weakSol,strongSol,'stable');
       if ~isempty(C)
           pos = C(1);
           weakPos = find(weakSol == pos);
           weakPos = weakPos(1);
           strongPos = find(strongSol == pos);
           strongPos = strongPos(1);
           weakSol(weakPos:end,:) = [];
           weakSol = [weakSol;strongSol(strongPos,:)];
           newpath = weakSol;
           newpath = removeLoops(newpath);
           Cost = size(newpath,1) + 100*pdist2(AllPoints(newpath(end,1),:),AllPoints(GoalNode,:));
       else
        spath1 = size(weakSol,1);
        spath2 = size(strongSol,1);
        pos1 = ceil(rand*spath1);
          
        newpath = weakSol(1:pos1,:);
        wNode = weakSol(pos1,1);
        cNode = wNode; 
        sNode = GoalNode;
        Mcounter = 100;
        counter = 0;
        while(cNode ~= sNode && counter <=Mcounter)
            pos2 = ceil(rand*spath2);
            sNode = strongSol(pos2,1);
            [tpath,cNode,~] = getPath(CollisionTest,wNode,sNode,AllPoints,ceil(nodes/2),Boundaryinitial,Boundaryfinal,s1,s2,s3,visited_nodes );
            counter = counter + 1;
        end
        if counter<= Mcounter
            newpath = [newpath;tpath(2:end,1)];
            [tpath,~,~] = getPath(CollisionTest,sNode,GoalNode,AllPoints,ceil(nodes/2),Boundaryinitial,Boundaryfinal,s1,s2,s3,visited_nodes );
            newpath = [newpath;tpath(2:end,1)];
        else 
            newpath = weakSol;
        end
        newpath = removeLoops(newpath);
        Cost = size(newpath,1) + 100*pdist2(AllPoints(newpath(end,1),:),AllPoints(GoalNode,:));
       end
   else
       Mcounter = 100;
       counter = 0;
       
       spath1 = size(weakSol,1);
       pos1 = ceil(rand*spath1);
       newpath = weakSol(1:pos1,:);
       tpath = [];
       wNode = weakSol(pos1,1);
       cNode = wNode; 
       sNode = GoalNode;
       
        while(cNode ~= sNode && counter <=Mcounter)
            sNode = ceil(rand*size(AllPoints,1));
            [tpath,cNode,~] = getPath(CollisionTest,wNode,sNode,AllPoints,ceil(nodes/2),Boundaryinitial,Boundaryfinal,s1,s2,s3,visited_nodes );
            counter = counter + 1;
        end
        if counter<= Mcounter && ~isempty(tpath)
            newpath = [newpath;tpath(2:end,1)];
            [tpath,~,~] = getPath(CollisionTest,sNode,GoalNode,AllPoints,ceil(nodes/2),Boundaryinitial,Boundaryfinal,s1,s2,s3,visited_nodes );
            newpath = [newpath;tpath(2:end,1)];
        else 
            newpath = weakSol;
        end
        newpath = removeLoops(newpath);
        Cost = size(newpath,1) + 100*pdist2(AllPoints(newpath(end,1),:),AllPoints(GoalNode,:));       
   end
end

