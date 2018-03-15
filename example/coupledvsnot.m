% This is to test the difference between decoupling lidar scans and not
% 1D toy example. Uses g2o because adding custom states to gtsam matlab sux

% For a coupled state, state is x and x'
clc;
clear;

% Ground truth is going to be a constant velocity increase
limit = 40;
vel_true = linspace(1, 10, limit);
%vel_true = ones(1, limit);
pos_true = cumsum(vel_true);

pos_noise_std = 0.2;
vel_noise_std = 0.02;

triangle = 0.5 * ones(1, limit);
triangle(1:2:end) = -triangle(1:2:end);

pos_corr = pos_true + pos_noise_std .* randn(1, limit) + triangle;
vel_corr = vel_true + vel_noise_std .* randn(1, limit);

figure(1);
clf;
plot(abs(pos_corr - pos_true));
hold on;

figure(2);
clf;
plot(abs(vel_corr - vel_true));
hold on;

% Now given corrupted measurements, find optimal solution

[ Graph ] = InitializeGraph;

dT = 1;
cutoff = limit;

motion_inf = 1000;
cv_inf = 1000;
del_inf = 100;

for i = 1:limit
    P_pos.value = pos_corr(i);
    P_pos.inf = 1/(pos_noise_std.^2);
    P_vel.value = vel_corr(i);
    P_vel.inf = 1/(vel_noise_std.^2);   
   
    if (i <= cutoff)
        [ Graph ] = AddUnaryEdge( Graph, 'PriorVector_Factor', 'Scalar', strcat('vel',num2str(i)), P_vel);
        [ Graph ] = AddUnaryEdge( Graph, 'PriorVector_Factor', 'Scalar', strcat('pos',num2str(i)), P_pos);
    end

    
    Graph.Nodes.Scalar.Values.(strcat('pos',num2str(i))) = P_pos.value;
    Graph.Nodes.Scalar.Values.(strcat('vel',num2str(i))) = P_vel.value;
    
    % now add motion factor
    if (i < limit)
        NodeArray = cell(3,2);
        NodeArray{1,1} = 'Scalar';
        NodeArray{1,2} = strcat('pos',num2str(i));
        NodeArray{2,1} = 'Scalar';
        NodeArray{2,2} = strcat('pos',num2str(i+1));
        NodeArray{3,1} = 'Scalar';
        NodeArray{3,2} = strcat('vel',num2str(i));
        
        Measurement_odom.value = dT;
        Measurement_odom.inf = motion_inf;        
        
        [ Graph ] = AddComplexEdge( Graph, 'OdomFactor', NodeArray, Measurement_odom);
        
        CV_meas.value = [];
        CV_meas.inf = cv_inf;
        
        NodeArray = cell(2,2);
        NodeArray{1,1} = 'Scalar';
        NodeArray{1,2} = strcat('vel',num2str(i));
        NodeArray{2,1} = 'Scalar';
        NodeArray{2,2} = strcat('vel',num2str(i+1));
        
        [ Graph ] = AddComplexEdge( Graph, 'ConstantScalar', NodeArray, CV_meas);
    end
end

[Solved] = PerformGO_DL(Graph);

pos_sol = zeros(1, limit);
vel_sol = zeros(1, limit);

for i=1:limit
    pos_sol(i) = Solved.Nodes.Scalar.Values.(strcat('pos', num2str(i)));
    vel_sol(i) = Solved.Nodes.Scalar.Values.(strcat('vel', num2str(i)));
end

figure(1)
plot(abs(pos_sol - pos_true));

figure(2)
plot(abs(vel_sol - vel_true));

% Now solve the same problem but with an extra degree of freedom
GraphMod = InitializeGraph;

for i = 1:limit
    P_pos.value = pos_corr(i);
    P_pos.inf = 1/(pos_noise_std.^2);
    P_vel.value = vel_corr(i);
    P_vel.inf = 1/(vel_noise_std.^2);
    
    NodeArray = cell(2,2);
    NodeArray{1,1} = 'Scalar';
    NodeArray{1,2} = strcat('pos',num2str(i));
    NodeArray{2,1} = 'Scalar';
    NodeArray{2,2} = strcat('del',num2str(i));
    
    if (i == 1)
        % Need a prior on the offset
        D.value = 0;
        D.inf = 0.001;
        [ GraphMod ] = AddUnaryEdge( GraphMod, 'PriorVector_Factor', 'Scalar', strcat('del',num2str(i)), D);
    end
    
    if (i <= cutoff)
        [ GraphMod ] = AddUnaryEdge( GraphMod, 'PriorVector_Factor', 'Scalar', strcat('vel',num2str(i)), P_vel);

        [ GraphMod ] = AddComplexEdge( GraphMod, 'LidarPrior', NodeArray, P_pos);
    end

    
    GraphMod.Nodes.Scalar.Values.(strcat('pos',num2str(i))) = P_pos.value;
    GraphMod.Nodes.Scalar.Values.(strcat('vel',num2str(i))) = P_vel.value;
    GraphMod.Nodes.Scalar.Values.(strcat('del',num2str(i))) = 0;
    
    % now add motion factor
    if (i < limit)
        NodeArray = cell(3,2);
        NodeArray{1,1} = 'Scalar';
        NodeArray{1,2} = strcat('pos',num2str(i));
        NodeArray{2,1} = 'Scalar';
        NodeArray{2,2} = strcat('pos',num2str(i+1));
        NodeArray{3,1} = 'Scalar';
        NodeArray{3,2} = strcat('vel',num2str(i));
        
        Measurement_odom.value = dT;
        Measurement_odom.inf = motion_inf;        
        
        [ GraphMod ] = AddComplexEdge( GraphMod, 'OdomFactor', NodeArray, Measurement_odom);
        
        CV_meas.value = [];
        CV_meas.inf = cv_inf;
        
        NodeArray = cell(2,2);
        NodeArray{1,1} = 'Scalar';
        NodeArray{1,2} = strcat('vel',num2str(i));
        NodeArray{2,1} = 'Scalar';
        NodeArray{2,2} = strcat('vel',num2str(i+1));
        
        [ GraphMod ] = AddComplexEdge( GraphMod, 'ConstantScalar', NodeArray, CV_meas);
        
        del_stiff.value = [];
        del_stiff.inf = del_inf;
        
        NodeArray = cell(2,2);
        NodeArray{1,1} = 'Scalar';
        NodeArray{1,2} = strcat('del',num2str(i));
        NodeArray{2,1} = 'Scalar';
        NodeArray{2,2} = strcat('del',num2str(i+1));
        
        [ GraphMod ] = AddComplexEdge( GraphMod, 'ConstantScalar', NodeArray, del_stiff);
    end
end

Solved2 = PerformGO_DL(GraphMod);

pos1_sol = zeros(1, limit);
vel1_sol = zeros(1, limit);
del_sol = zeros(1, limit);

for i=1:limit
    pos1_sol(i) = Solved2.Nodes.Scalar.Values.(strcat('pos', num2str(i)));
    vel1_sol(i) = Solved2.Nodes.Scalar.Values.(strcat('vel', num2str(i)));
    del_sol(i) = Solved2.Nodes.Scalar.Values.(strcat('del', num2str(i)));
end

figure(3)
clf;
hold on;
plot(pos1_sol);
plot(vel1_sol);
plot(del_sol);
plot(pos_true);
plot(vel_true);
legend('position', 'velocity', 'offset', 'true_pos', 'true_vel');

figure(1);
plot(abs(pos1_sol + del_sol - pos_true));
legend('meas_err', 'coupl_err', 'noncoupl_err');

figure(2);
plot(abs(vel1_sol - vel_true));
legend('meas_err', 'coupl_err', 'noncoupl_err');