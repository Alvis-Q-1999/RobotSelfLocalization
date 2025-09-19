function main_pf_mod()
clc;
clear all;
cla;


addpath('PSO_PF/core');
addpath('PSO_PF/viz');
addpath('PSO_PF/map');

%% 显示设置
colorshape_real = 'bd';
color_real = 'b';
colorshape_p1 = 'm*';
color_lbest = [0.25, 0.5, 0.8];
colorshape_resampling = 'y*';
markersize = 4;
markersize2 = 3;
pause_time = 0.05;


load('map_l_1.mat');                  % 变量 G
MM = size(G,1);                    % 行列数（正方形）
d  = 2;                            % 维度

figure(1);
plotMap(G);                        % 黑=占用(1), 白=空闲(0)
grid on; set(gca,'Layer','top'); axis([0,MM,0,MM]); hold on;

%% 初始化粒子/空闲/障碍
[P_pos, free_spot, wall_spot, N] = initParticles(G);

pause(0.1);

N

%机器人真实位置
% P_pos_real = GetRandomPos(N, free_spot) 
% P_pos_real = [22,13]; %H1
P_pos_real = [68,29]; %L1
% P_pos_real = [11,41]; %R1
% P_pos_real = [72,82]; %O1
% P_pos_real = [61,79];
% P_pos_real = [95,83];

%% 初始位置估计
%% Step1 粒子初始化

w = 0.87; % 惯性权重
c_1 = 0.5;% 自我学习因子
c_2 = 0.5;% 群体学习因子 
v = rand(N, d);% 初始种群的速度
vlimit = [-1.5, 1.5; -1.5, 1.5];% 设置速度限制
xlimit = [0.51, MM-0.51; 0.51, MM-0.51];

P_fitness = -inf(N, 1);% 个体当前适应度
P_best_pos = P_pos;% 个体最优位置
P_best_fitness = -inf(N, 1);% 个体最优适应度

L_best_num = 0;% 邻域最优个数
L_best_threshold = 0.03;% 成为邻域最优的最低阈值
L_best_pos = zeros(N, d);% 邻域最优位置
P_min_distance = ones(N,1)*MM;

%显示
% pos = gridCoord(P_pos, MM);
% p1 = plot(pos(1,:), pos(2,:),colorshape_p1,'markersize',markersize);
% pos_R = gridCoord(P_pos_real, MM);
% pr = plot(pos_R(1,:), pos_R(2,:),colorshape_real,'MarkerFaceColor',color_real);
% 
% set(gca,'Layer','top');
% pause(pause_time);
% 
% delete(p1);

[hParticles, ~, hReal] = plotParticles(G, P_pos, [], P_pos_real);
pause(pause_time);
delete(hParticles);



%% Step2 PSO

ger1 = 20;% 初始位置预测pso迭代次数
real_scan_result = scan8dirs(G, P_pos_real);
real_fitness = fitness(real_scan_result, real_scan_result)

for iter1 = 1 : ger1
    % 获取适应度
    for i = 1:N
        P_scan_result = scan8dirs(G, P_pos(i,:));
        P_fitness(i) =  fitness(real_scan_result, P_scan_result);
    end
    
    % 更新最佳适应度
    for i = 1:N
        if P_best_fitness(i) < P_fitness(i)
            P_best_fitness(i) = P_fitness(i); % 更新个体历史最佳适应度
            P_best_pos(i,:) = P_pos(i,:);% 更新个体历史最佳位置
        end 
    end
%     
    
    if mod(iter1, 10) == 0 || iter1 == 1

        % 获取邻域最优解
        for i = 1:N
            if abs(P_best_fitness(i) - real_fitness) < L_best_threshold
                L_best_num = L_best_num + 1;
                Top_best_pos(L_best_num, :) = P_best_pos(i, :);
            end
        end
        L_best_threshold = L_best_threshold / 5;
%     else
%         Top_best_pos = Top_best_pos;
    end
    
    % 匹配邻域最优解
    for i = 1:N
        for j = 1:L_best_num
                pdistance = euclid(P_pos(i,:),Top_best_pos(j,:));
                if pdistance < P_min_distance(i)
                    L_best_pos(i,:) = Top_best_pos(j,:);
                    P_min_distance(i) = pdistance;
                end
        end
    end
    
    % 更新速度
    v = v * w + c_1 * rand *(P_best_pos - P_pos) + c_2 * rand *(L_best_pos - P_pos);
%     v = clamp(v, vlimit, d);% 边界位置处理
    

    % 更新位置
    P_pos_N = P_pos + v;
     P_pos_N = clamp(P_pos_N, xlimit, d);% 边界位置处理
    
    % 避障处理1
    P_pos_N_R = round(P_pos_N);    
    for i = 1:N 
        if G(P_pos_N_R(i,1),P_pos_N_R(i,2)) == 0
            P_pos(i,:) = P_pos_N(i,:);
        else
            P_pos(i,:) = P_pos(i,:);
        end
    end
    
    %显示
    [hParticles, hTop, hReal] = plotParticles(G, P_pos, Top_best_pos, P_pos_real);
    pause(pause_time);
    delete(hParticles);
    delete(hTop);
    %迭代次数
end

%% Step3 重采样
min_distance_threshold = 2;
for i = 1:N
    if P_min_distance(i) > min_distance_threshold
        P_pos(i,:) = L_best_pos(i,:);
        P_best_pos(i,:) = P_pos(i,:);
        P_scan_result = scan8dirs(G, P_pos(i,:));
        P_fitness(i) =  fitness(real_scan_result, P_scan_result);
        P_best_fitness(i) = P_fitness(i);
    end
end


% 显示

[hParticles, hTop, hReal] = plotParticles(G, P_pos, Top_best_pos, P_pos_real);


pause(pause_time);
delete(hParticles);
delete(hTop);

%% DBSCAN
epsilon = 3;
min_points = 20;

visited=false(N,1);
isnoise=false(N,1);
noises_id = 0;
subswarm_N = 0;
P_subswarm_id = zeros(N,1);
cluster_distance = pdist2(P_pos, P_pos);

for i = 1:N
    if ~visited(i)
        visited(i) = true;
        neighbors=regionQuery(i,cluster_distance,epsilon);
        if numel(neighbors)<min_points
            % X(i,:) is NOISE
            isnoise(i)=true;
            noises_id =noises_id + 1;
            noises(noises_id) = i;
        else
            subswarm_N = subswarm_N + 1;
            s_P_N = 1;
            subswarms(subswarm_N, s_P_N) = i;
            s_P_N = s_P_N + 1;
            P_subswarm_id(i) = subswarm_N;
            k = 1;
            while true
                j = neighbors(k);
                if ~visited(j)
                    visited(j)=true;
                    neighbors2=regionQuery(j,cluster_distance,epsilon);
                    num_neighbors2 = numel(neighbors2);
                    if num_neighbors2>=min_points
                        neighbors=[neighbors neighbors2];   %#ok
                    end
                end
                if P_subswarm_id(j) == 0
                    P_subswarm_id(j) = subswarm_N;
                    subswarms(subswarm_N, s_P_N) = j;
                    s_P_N = s_P_N + 1;
                end
                k = k + 1;
                if k > numel(neighbors)
                    break;
                end
            end
        end
    end    
end

for i = 1 : noises_id
    rand_1 = randi([1,subswarm_N],1);
    subswarm_current = subswarms(rand_1,:);    
    subswarm_current(subswarm_current==0)=[];
    s_N = size(subswarm_current,2);
    I_copy =  randi([1,s_N],1);
    I = noises(i);
    P_pos(I,:) = P_pos(I_copy,:);
    P_best_pos(I,:) = P_best_pos(I_copy,:);
    P_fitness(I) =  P_fitness(I_copy);
    P_best_fitness(I) = P_fitness(I_copy);
end

subswarm_N
color_list_c  = rand(subswarm_N, 3);
% 显示
for i = 1 : subswarm_N
    subswarm_current = subswarms(i,:);
    subswarm_current(subswarm_current==0)=[];
    s_P_pos = P_pos(subswarm_current,:);
    pos = gridCoord(s_P_pos, MM);
    p2 = plot(pos(1,:), pos(2,:),'*','Color',color_list_c(i,:),'markersize',markersize);
    
end

[~, hTop, hReal] = plotParticles(G, [], Top_best_pos, P_pos_real);

pause(pause_time);

delete(hTop);
 

%% 姿态追踪
[dirIdx, baseDir] = chooseOpenDirection(G, P_pos_real);
%% 移动
ger3 = 20;
dead_N = 0;
for iter3 =1 : ger3
    
%     v_step = biasedConeWalker(G, P_pos_real, [1 10], 4, 1.5);
    v_step = biasedRandomStep(baseDir, [4 10], 0.5);   % 长度1~6，20%侧偏

    
    P_pos_real = moveRealWithObstacles(G, P_pos_real, v_step);
    P_pos_real = clamp(P_pos_real, xlimit, d)
    
%     stepx = v_real(iter3,1)/abs(v_real(iter3,1));
%     stepy = v_real(iter3,2)/abs(v_real(iter3,2));
%     for i = 1:N 
%         for x = 1:v_real(iter3,1)
%             P_x = P_pos(i,1);
%             P_x_round = round(P_x) + stepx; 
%             P_y_round = round(P_pos(i,2));
%             if P_x_round >= MM || P_x_round <= 1
%                 break;
%             elseif G(P_x_round,P_y_round) == 1
%                 break;
%             else
%                 P_pos(i,1) = P_x + stepx + rand(1,1)*0.5;
%             end
%         end
%         
%         for y = 1:v_real(iter3,2)
%             P_y= P_pos(i,2) ;
%             P_x_round = round(P_pos(i,1)); 
%             P_y_round = round(P_y) + stepy;
%             if P_y_round >= MM || P_y_round <= 1
%                 break;
%             elseif G(P_x_round,P_y_round) == 1
%                 break;
%             else
%                 P_pos(i,2) = P_y + stepy + rand(1,1)*0.5;
%             end
%         end
%     end

    stepx = sign(v_step(1)); stepy = sign(v_step(2));
        for i = 1:N
            for x = 1:abs(v_step(1))
                P_x = P_pos(i,1);
                P_x_round = round(P_x) + stepx;
                P_y_round = round(P_pos(i,2));
                if P_x_round >= MM || P_x_round <= 1, break;
                elseif G(P_x_round, P_y_round) == 1, break;
                else, P_pos(i,1) = P_x + stepx + rand*0.5;
                end
            end
            for y = 1:abs(v_step(2))
                P_y = P_pos(i,2);
                P_x_round = round(P_pos(i,1));
                P_y_round = round(P_y) + stepy;
                if P_y_round >= MM || P_y_round <= 1, break;
                elseif G(P_x_round, P_y_round) == 1, break;
                else, P_pos(i,2) = P_y + stepy + rand*0.5;
                end
            end
        end
    P_best_pos = P_pos;% 个体最优位置 
    P_best_fitness = -inf(N, 1);% 个体最优适应度
    % 显示新的真实位置

    cla reset; 
    plotMap(G);
    
    min_distance_threshold = 2;
    %% 子群内部
    real_scan_result = scan8dirs(G, P_pos_real);
    real_fitness = fitness(real_scan_result, real_scan_result);
    subswarm_fitness = -inf(subswarm_N,1);
    for s = 1 : subswarm_N
        %子群内部数据    
        subswarm_current = subswarms(s,:);    
        subswarm_current(subswarm_current==0)=[];% 该子群粒子在P_pso中的编号
        s_P_pos = P_pos(subswarm_current,:);% 该子群粒子位置
        s_N = size(subswarm_current, 2);% 该子群粒子个数

        s_P_fitness = -inf(s_N, 1);
        s_P_best_pos = s_P_pos;
        s_P_best_fitness = -inf(s_N, 1);

        s_G_best_pos = zeros(1, d);
        s_G_best_fitness = -inf;

        % 获取适应度
        for i = 1:s_N
            I = subswarm_current(i);
            s_P_scan_result = scan8dirs(G, s_P_pos(i,:));
            s_P_fitness(i) =  fitness(real_scan_result, s_P_scan_result);
            P_fitness(I) = s_P_fitness(i);
        end

        % 更新最佳适应度
        for i = 1:s_N
            I = subswarm_current(i);
            if s_P_best_fitness(i) < s_P_fitness(i)
                s_P_best_fitness(i) = s_P_fitness(i); % 更新个体历史最佳适应度
                s_P_best_pos(i,:) = s_P_pos(i,:);
                P_best_fitness(I) = s_P_fitness(i);
                P_best_pos(I,:) = s_P_pos(i,:);
                % 更新个体历史最佳位置
            end
        end
        if s_G_best_fitness  < max(s_P_fitness)
            [s_G_best_fitness, max_pos] = max(s_P_fitness);   % 更新群体历史最佳适应度
            s_G_best_pos = s_P_pos(max_pos, :);
            L_best_pos(:,1) = s_G_best_pos(1);
            L_best_pos(:,2) = s_G_best_pos(2);
        end
        
        %重采样
        for i = 1:s_N 
            I = subswarm_current(i);
            pdistance = euclid(s_P_pos(i,:),s_G_best_pos);
            if pdistance > min_distance_threshold %=2
                s_P_pos(i,:) = s_G_best_pos;
                s_P_best_pos(i,:) = s_P_pos(i,:);
                s_P_fitness(i) =  s_G_best_fitness;
                s_P_best_fitness(i) = s_P_fitness(i);
                P_pos(I,:) = s_P_pos(i,:);
                P_best_pos(I,:) = s_P_best_pos(i,:);
                P_fitness(I) =  s_P_fitness(i);
                P_best_fitness(I) = s_P_fitness(i);
            end
        end
        [hParticles, hTop, hReal] = plotParticles(G, P_pos, Top_best_pos, P_pos_real);
        

        subswarm_fitness(s) = mean(s_P_best_fitness);
        
    end

    % 画图
    cla reset; 
    plotMap(G);

    subswarm_fitness
    subswarm_fitness_threshold = 0.9;
    %% Step3 重采样
    new_subswarm_N = 0;
    keep_flag = false(subswarm_N,1);
    for s = 1 : subswarm_N 
        subswarm_current = subswarms(s,:);
        subswarm_current(subswarm_current==0)=[];
        s_N = size(subswarm_current, 2);
        if subswarm_fitness(s) >= subswarm_fitness_threshold
            new_subswarm_N = new_subswarm_N + 1;
            new_subswarms(new_subswarm_N,[1:s_N]) = subswarm_current;
            keep_flag(s)  = true;
        end
    end
    for s = 1 : subswarm_N
       if ~keep_flag(s)%被淘汰
            subswarm_current = subswarms(s,:);
            subswarm_current(subswarm_current==0)=[];
            s_N = size(subswarm_current, 2);
            if new_subswarm_N == 0
                break;
            elseif new_subswarm_N == 1
                subswarm_new_current = new_subswarms(1,:);
                subswarm_new_current(subswarm_new_current==0)=[];
                for i = 1 : s_N
                    I = subswarm_current(i);
                    dead_N = dead_N + 1;%记录淘汰粒子
                    dead_P(dead_N,:) = P_pos(I,:);
                    seed = size(subswarm_new_current,2);
                    j = randi([1,seed]);
                    I_copy = new_subswarms(1,j);
                    P_pos(I,:) = P_pos(I_copy,:);
                    P_fitness(I) =  P_fitness(I_copy);
                end
            elseif new_subswarm_N > 1
                for i = 1 : s_N
                    I = subswarm_current(i);
                    dead_N = dead_N + 1;
                    dead_P(dead_N,:) = P_pos(I,:);
                    k = randi([1,new_subswarm_N]);
                    subswarm_new_current = new_subswarms(k,:);
                    subswarm_new_current(subswarm_new_current==0)=[];
                    seed = size(subswarm_new_current,2);
                    j = randi([1,seed]);
                    I_copy = new_subswarms(k,j);
                    P_pos(I,:) = P_pos(I_copy,:);
                    P_fitness(I) =  P_fitness(I_copy);
                end
            end
        end
    end
    
    subswarm_fitness_threshold = subswarm_fitness_threshold*1.01;
    
    %% DBSCAN
    clear noise;
    epsilon = 2;
    min_points = 50;

    visited=false(N,1);
    isnoise=false(N,1);
    noises_id = 0;
    subswarm_N = 0;
    P_subswarm_id = zeros(N,1);
    cluster_distance = pdist2(P_pos, P_pos);
    clear neighbors;
    clear neighbors2;
    clear subswarms;
    for i = 1:N
        if ~visited(i)
            visited(i) = true;
            neighbors=regionQuery(i,cluster_distance,epsilon);
            if numel(neighbors)<min_points
                % X(i,:) is NOISE
                isnoise(i)=true;
                noises_id =noises_id + 1;
                noises(noises_id) = i;
            else
                subswarm_N = subswarm_N + 1;
                s_P_N = 1;
                subswarms(subswarm_N, s_P_N) = i;
                s_P_N = s_P_N + 1;
                P_subswarm_id(i) = subswarm_N;
                k = 1;
                while true
                    j = neighbors(k);
                    if ~visited(j)
                        visited(j)=true;
                        neighbors2=regionQuery(j,cluster_distance,epsilon);
                        num_neighbors2 = numel(neighbors2);
                        if num_neighbors2>=min_points
                            neighbors=[neighbors neighbors2];   %#ok
                        end
                    end
                    if P_subswarm_id(j) == 0
                        P_subswarm_id(j) = subswarm_N;
                        subswarms(subswarm_N, s_P_N) = j;
                        s_P_N = s_P_N + 1;
                    end
                    k = k + 1;
                    if k > numel(neighbors)
                        break;
                    end
                end
            end
        end    
    end

    
    for i = 1 : noises_id
        rand_1 = randi([1,subswarm_N],1);
        subswarm_current = subswarms(rand_1,:);    
        subswarm_current(subswarm_current==0)=[];
        s_N = size(subswarm_current,2);
        I_copy =  randi([1,s_N],1);
        I = noises(i);
        P_pos(I,:) = P_pos(I_copy,:);
        P_best_pos(I,:) = P_best_pos(I_copy,:);
        P_fitness(I) =  P_fitness(I_copy);
        P_best_fitness(I) = P_fitness(I_copy);
    end

    
    color_list_c  = rand(subswarm_N, 3);
%     %淘汰粒子
%     for i = 1 : dead_N
%         pos_D = gridCoord(dead_P,MM);
%         pd = plot(pos_D(1,:), pos_D(2,:),'*','Color',[0.5,0.5,0.5],'markersize',markersize);
%     end
    for i = 1 : subswarm_N
        subswarm_current = subswarms(i,:);
        subswarm_current(subswarm_current==0)=[];
        s_P_pos = P_pos(subswarm_current,:);
        pos = gridCoord(s_P_pos, MM);
        p2 = plot(pos(1,:), pos(2,:),'*','Color',color_list_c(i,:),'markersize',markersize);
    end
    pos_R = gridCoord(P_pos_real, MM);
    pr = plot(pos_R(1,:), pos_R(2,:),colorshape_real,'MarkerFaceColor',color_real);
    pause(pause_time);

    subswarm_N
    if subswarm_N ==1
        break;
    end
end
end






function pos = GetRandomPos(N, free_spot)
n = randi([1,N],1,1);
pos = free_spot(n,:);
end


