function [ desired_state ] = trajectory_generator(t, vquad, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.

persistent map0 path0 allequations;

switch nargin
    case 4
        path0 = path;
        map0 = map;
        for i = 1:size(path,2)
            fileName = strcat('allequations',num2str(i));
            fileName = strcat(fileName,'.mat');
            load(fileName);
            allequations{i} = equations;
        end
        return;
    case {2,3}

% for t = 1:0.05:tmax 
        times = allequations{vquad}(:,8);
        tmax = max(allequations{vquad}(:,8));
        equationindex = find(times>=t, 1 );
        if t == 0
            pos = path0{vquad}(1,:)';
            vel = [0 0 0]';
            acc = [0 0 0]';
        elseif t>0 && t<tmax
%             coeffpos_x = sym2poly(allequations{equationindex,1});
%             if numel(coeffpos_x) == 1
%                 pos_x = coeffpos_x;
%             else
%                 pos_x = coeffpos_x(1)*t^5 + coeffpos_x(2)*t^4 + coeffpos_x(3)*t^3 + coeffpos_x(4)*t^2 + coeffpos_x(5)*t + coeffpos_x(6);
%             end
%             coeffvel_x = sym2poly(allequations{equationindex,2});
%             if numel(coeffvel_x) == 1
%                 vel_x = 0;
%             else
%                 vel_x = coeffvel_x(1)*t^4 + coeffvel_x(2)*t^3 + coeffvel_x(3)*t^2;
%             end
%             coeffacc_x = sym2poly(allequations{equationindex,3});
%             if numel(coeffacc_x) == 1
%                 acc_x = 0;
%             else
%                 acc_x = coeffacc_x(1)*t^3 + coeffacc_x(2)*t^2 + coeffacc_x(3)*t^1;
%             end
% 
%             coeffpos_y = sym2poly(allequations{equationindex+1,1});
%             if numel(coeffpos_y) == 1
%                 pos_y = coeffpos_y;
%             else
%                 pos_y = coeffpos_y(1)*t^5 + coeffpos_y(2)*t^4 + coeffpos_y(3)*t^3 + coeffpos_y(4)*t^2 + coeffpos_y(5)*t + coeffpos_y(6);
%             end
%             coeffvel_y = sym2poly(allequations{equationindex+1,2});
%             if numel(coeffvel_y) == 1
%                 vel_y = 0;
%             else
%                 vel_y = coeffvel_y(1)*t^4 + coeffvel_y(2)*t^3 + coeffvel_y(3)*t^2;
%             end
%             coeffacc_y = sym2poly(allequations{equationindex+1,3});
%             if numel(coeffacc_y) == 1
%                 acc_y = 0;
%             else
%                 acc_y = coeffacc_y(1)*t^3 + coeffacc_y(2)*t^2 + coeffacc_y(3)*t^1;
%             end
% 
%             coeffpos_z = sym2poly(allequations{equationindex+2,1});
%             if numel(coeffpos_z) == 1
%                 pos_z = coeffpos_z;
%             else
%                 pos_z = coeffpos_z(1)*t^5 + coeffpos_z(2)*t^4 + coeffpos_z(3)*t^3 + coeffpos_z(4)*t^2 + coeffpos_z(5)*t + coeffpos_z(6);
%             end
%             coeffvel_z = sym2poly(allequations{equationindex+2,2});
%             if numel(coeffvel_z) == 1
%                 vel_z = 0;
%             else
%                 vel_z = coeffvel_z(1)*t^4 + coeffvel_z(2)*t^3 + coeffvel_z(3)*t^2;
%             end
%             coeffacc_z = sym2poly(allequations{equationindex+2,3});
%             if numel(coeffacc_z) == 1
%                 acc_z = 0;
%             else
%                 acc_z = coeffacc_z(1)*t^3 + coeffacc_z(2)*t^2 + coeffacc_z(3)*t^1;
%             end
            Cx = allequations{vquad}(equationindex,1:6);
            pos_x = Cx(1) + Cx(2)*t + Cx(3)*t^2 + Cx(4)*t^3 + Cx(5)*t^4 + Cx(6)*t^5;
            vel_x = 0 + Cx(2) + 2*Cx(3)*t + 3*Cx(4)*t^2 + 4*Cx(5)*t^3 + 5*Cx(6)*t^4;
            acc_x = 0 + 0 + 2*Cx(3) + 2*3*Cx(4)*t + 3*4*Cx(5)*t^2 + 4*5*Cx(6)*t^3;
            
            Cy = allequations{vquad}(equationindex+1,1:6);
            pos_y = Cy(1) + Cy(2)*t + Cy(3)*t^2 + Cy(4)*t^3 + Cy(5)*t^4 + Cy(6)*t^5;
            vel_y = 0 + Cy(2) + 2*Cy(3)*t + 3*Cy(4)*t^2 + 4*Cy(5)*t^3 + 5*Cy(6)*t^4;
            acc_y = 0 + 0 + 2*Cy(3) + 2*3*Cy(4)*t + 3*4*Cy(5)*t^2 + 4*5*Cy(6)*t^3;
            
            Cz = allequations{vquad}(equationindex+2,1:6);
            pos_z = Cz(1) + Cz(2)*t + Cz(3)*t^2 + Cz(4)*t^3 + Cz(5)*t^4 + Cz(6)*t^5;
            vel_z = 0 + Cz(2) + 2*Cz(3)*t + 3*Cz(4)*t^2 + 4*Cz(5)*t^3 + 5*Cz(6)*t^4;
            acc_z = 0 + 0 + 2*Cz(3) + 2*3*Cz(4)*t + 3*4*Cz(5)*t^2 + 4*5*Cz(6)*t^3;

            pos = [pos_x pos_y pos_z]';
            vel = [vel_x vel_y vel_z]';
            acc = [acc_x acc_y acc_z]';
            
        elseif t>tmax
            pos = path0{vquad}(end,:)';
            vel = [0 0 0]';
            acc = [0 0 0]';
        end
end
% end

%     switch nargin
%         case 4
%     %         map0 = map;
%     %         path0 = path;
%     %         pos(1) = 0;
%     %         vel(1) = 0;
%     %         acc(1) = 0;
%     %         
%     %         pos(2) = (63*t^5)/160 - (63*t^4)/32 + (21*t^3)/8 - 49/10;
%     %         vel(2) = (63*t^4)/32 - (63*t^3)/8 + (63*t^2)/8;
%     %         acc(2) = (63*t^3)/8 - (189*t^2)/8 + (63*t)/4;
%     %         
%     %         pos(3) = (27*t^5)/80 - (27*t^4)/16 + (9*t^3)/4 + 1/5;
%     %         vel(3) = (27*t^4)/16 - (27*t^3)/4 + (27*t^2)/4;
%     %         acc(3) = (27*t^3)/4 - (81*t^2)/4 + (27*t)/2;
%         case 2
%             pos(1) = (9*t^5)/25000 - (9*t^4)/1000 + (3*t^3)/50;
%             vel(1) = (9*t^4)/5000 - (9*t^3)/250 + (9*t^2)/50;
%             acc(1) = (9*t^3)/1250 - (27*t^2)/250 + (9*t)/25;
% 
%             pos(2) = (99007134208113*t^5)/72057594037927936 - (687*t^4)/20000 + (229*t^3)/1000 - 49/10;
%             vel(2) = (495035671040565*t^4)/72057594037927936 - (687*t^3)/5000 + (687*t^2)/1000;
%             acc(2) = (495035671040565*t^3)/18014398509481984 - (2061*t^2)/5000 + (687*t)/500;
% 
%             pos(3) = (6198106008766411*t^5)/36893488147419103232 - (21*t^4)/5000 + (7*t^3)/250 + 1/5;
%             vel(3) = (30990530043832055*t^4)/36893488147419103232 - (21*t^3)/1250 + (21*t^2)/250;
%             acc(3) = (30990530043832055*t^3)/9223372036854775808 - (63*t^2)/1250 + (21*t)/125;
%     end
% end

yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end

