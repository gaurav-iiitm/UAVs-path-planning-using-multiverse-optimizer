function [ paths,CurrentNode,NVNodes ] = getPath(CollisionTest, StartNode,GoalNode,AllPoints,nodes,Boundaryinitial,Boundaryfinal,s1,s2,s3,visited_nodes )
%GETPATH Summary of this function goes here
%   Detailed explanation goes here

    












MaxNoofNodes = size(AllPoints,1);
    NodesEvaluated = zeros(MaxNoofNodes,1);
    if size(visited_nodes,1) > 1
        NodesEvaluated(visited_nodes) = 1;
    end
    NodesEvaluated(StartNode) = 1;
    CurrentNode = StartNode;
    paths = StartNode;
    curNodeIndex = 1;
    NVNodes = 0;
    prevNode = GoalNode;
    while le(NVNodes,nodes) && ~isequal(CurrentNode,GoalNode)
        hor = AllPoints(CurrentNode,1)>AllPoints(GoalNode,1);
        ver = AllPoints(CurrentNode,2)>AllPoints(GoalNode,2);
        plan = AllPoints(CurrentNode,3)>AllPoints(GoalNode,3);
        if(isequal(AllPoints(CurrentNode,1),AllPoints(GoalNode,1)))
            hor = 2;
        end
        if(isequal(AllPoints(CurrentNode,2),AllPoints(GoalNode,2)))
            ver = 2;
        end
        if(isequal(AllPoints(CurrentNode,3),AllPoints(GoalNode,3)))
            plan = 2;
        end
        directions = [ver;hor;plan];
        [CurrentNeighbours,ViableNeighbours] = Neighbours(CurrentNode,s1,s2,s3, Boundaryinitial, Boundaryfinal,AllPoints,directions);
        CurrentNeighbours = CurrentNeighbours(~CollisionTest(CurrentNeighbours));
        ViableNeighbours = ViableNeighbours(~CollisionTest(ViableNeighbours));
        CurrentNeighboursABS = CurrentNeighbours(NodesEvaluated(CurrentNeighbours) == 0 );
        ViableNeighbours = ViableNeighbours(ViableNeighbours ~= prevNode);
        ViableNeighboursABS = ViableNeighbours(NodesEvaluated(ViableNeighbours) == 0);
        flag = 0;
        if ~isempty(ViableNeighboursABS)
            pos = ceil(rand*size(ViableNeighboursABS,1));
            pos = ViableNeighboursABS(pos);
        elseif isempty(ViableNeighboursABS) && ~isempty(CurrentNeighboursABS)
            pos = ceil(rand*size(CurrentNeighboursABS,1));
            pos = CurrentNeighboursABS(pos);
            flag = 1;
        elseif isempty(CurrentNeighboursABS) && ~isempty(ViableNeighbours)
            pos = ceil(rand*size(ViableNeighbours,1));
            pos = ViableNeighbours(pos);
        elseif isempty(ViableNeighbours)
            pos = ceil(rand*size(CurrentNeighbours,1));
            pos = CurrentNeighbours(pos);
            flag = 1;
        else 
            break;
        end
            if flag == 1 
                NVNodes = NVNodes + 1; 
                %fprintf('%d %d %d\n',AllPoints(pos,:));
            end
            paths = [paths;pos];
            NodesEvaluated(pos) = 1;
            prevNode = CurrentNode;
            CurrentNode = pos;  
            curNodeIndex = curNodeIndex + 1;
        %fprintf('%d %d %d\n',curNodeIndex,nodes,~isequal(CurrentNode,GoalNode));
    end


end

