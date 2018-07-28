function path = removeLoops(Paths)
    path = Paths(:,:);
    psize = size(path,1);
    i = 1;
    while i <= psize    
        cur = path(i,:);
        for j = i+1:1:psize
            if cur(1,:) == path(j,:)
                path(i+1:j,:) = [];
                psize = size(path,1);
                break;
            end
        end
        i = i + 1; 
    end
    
end