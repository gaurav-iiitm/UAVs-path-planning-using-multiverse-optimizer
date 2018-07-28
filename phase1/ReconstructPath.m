function Path = ReconstructPath(PreviousNodes,CurrentNode)
% CurrentNode = 571;
Path = CurrentNode;
while CurrentNode~=0
    CurrentNode = PreviousNodes(CurrentNode);
    Path = [Path;CurrentNode];
end
Path = Path(Path>0);
Path = flipud(Path);
% Path = AllPoints(Path);
end

    
