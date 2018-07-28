function CostEstimate = Heuristic(point,GoalNode,AllPoints,astar)
if astar==true
%     CostEstimate = sum(abs(AllPoints(point,:)-AllPoints(GoalNode,:)));
%     CostEstimate = pdist2(AllPoints(point, :),AllPoints(GoalNode, :));
    
     CostEstimate=sqrt(sum(bsxfun(@minus, AllPoints(point, :), AllPoints(GoalNode,:)).^2, 2));
else
% CostEstimate = pdist([point;goal]);
    CostEstimate = 0;
end
end