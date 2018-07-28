load allequations.mat
allequations = equations;
tmax = 15;
for t =1:0.05:tmax
    a = subs(allequations{equationindex+1,1})
%     b = sym2poly(allequations{equationindex+1,1})
%     c = vpa(allequations{equationindex+1,1})
%     subs(a,t);
%     subs(b,t);
%     subs(c,t);
%     
end