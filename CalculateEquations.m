function [CurrentEquation] = CalculateEquations(currentstart,currentstop,tmin,tmax)
% syms t
% currentstart = [0 -4.9 0.2];
% currentstop = [0 -2.9 0];
% tmin = 1;
% tmax = 2;
Ax = [1 tmin tmin^2 tmin^3 tmin^4 tmin^5; 0 1 2*tmin 3*tmin^2 4*tmin^3 5*tmin^4;...
    0 0 2 6*tmin 12*tmin^2 20*tmin^3;1 tmax tmax^2 tmax^3 tmax^4 tmax^5;...
    0 1 2*tmax 3*tmax^2 4*tmax^3 5*tmax^4; 0 0 2 6*tmax 12*tmax^2 20*tmax^3 ];
Cx = (Ax)\[currentstart(1) 0 0 currentstop(1) 0 0]';
% x = C(1) + C(2)*t + C(3)*t^2 + C(4)*t^3 + C(5)*t^4 +  C(6)*t^5;
% xd = diff(x,t);
% xdd = diff(xd,t);
CurrentEquation = [Cx' tmin tmax];

Ay = [1 tmin tmin^2 tmin^3 tmin^4 tmin^5; 0 1 2*tmin 3*tmin^2 4*tmin^3 5*tmin^4;...
    0 0 2 6*tmin 12*tmin^2 20*tmin^3;1 tmax tmax^2 tmax^3 tmax^4 tmax^5;...
    0 1 2*tmax 3*tmax^2 4*tmax^3 5*tmax^4; 0 0 2 6*tmax 12*tmax^2 20*tmax^3 ];
Cy = (Ay)\[currentstart(2) 0 0 currentstop(2) 0 0]';
% y = C(1) + C(2)*t + C(3)*t^2 + C(4)*t^3 + C(5)*t^4 +  C(6)*t^5;
% yd = diff(y,t);
% ydd = diff(yd,t);
CurrentEquation(2,:) = [Cy' tmin tmax];

Az = [1 tmin tmin^2 tmin^3 tmin^4 tmin^5; 0 1 2*tmin 3*tmin^2 4*tmin^3 5*tmin^4;...
    0 0 2 6*tmin 12*tmin^2 20*tmin^3;1 tmax tmax^2 tmax^3 tmax^4 tmax^5;...
    0 1 2*tmax 3*tmax^2 4*tmax^3 5*tmax^4; 0 0 2 6*tmax 12*tmax^2 20*tmax^3 ];
Cz = (Az)\[currentstart(3) 0 0 currentstop(3) 0 0]';
% z = C(1) + C(2)*t + C(3)*t^2 + C(4)*t^3 + C(5)*t^4 +  C(6)*t^5;
% zd = diff(z,t);
% zdd = diff(zd,t);
CurrentEquation(3,:) = [Cz' tmin tmax];
end