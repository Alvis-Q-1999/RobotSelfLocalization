function main_final()
clc;
clear all;
cla;

%% 显示设置
colorshape_real = 'bd';
color_real = 'b';
colorshape_p1 = 'm*';
color_lbest = [0.25, 0.5, 0.8];
colorshape_resampling = 'y*';
markersize = 4;
markersize2 = 3;
pause_time = 0.3;

% 加载地图
load MapT.mat;
MM=size(G,1);%行列数（必须是正方形）
d=2;%维度
grid on 
set(gca,'Layer','top');
figure(1) 
axis([0,MM,0,MM]) 
W = 0; color_W = 'k'; % 占用栅格数-黑色代表占用
F = 0; color_F = 'w'; % 空闲栅格数-白色代表空闲

N = 0; % 粒子数

% 画图
for i=1:MM
    for j=1:MM
        if G(i,j) == 1 %黑色代表占用
            %占用区域记录
            W = W + 1;
            wall_spot(W,1) = i;
            wall_spot(W,2) = j;
            % 画图
            SetColor(wall_spot(W,:), color_W, G);
            hold on
        else %白色代表空闲
            %粒子初始化
            N = N + 1; %粒子数
            P_pos(N,1) = i;%个体当前位置
            P_pos(N,2) = j;
            %空闲区域记录
            F = F + 1; 
            free_spot(F,1) = i;
            free_spot(F,2) = j;
            %P_pos(N,3)=1;%旋转角度 1-从正左，2-从左上，3-从正上，...，8-从左下
            % 画图
            SetColor(free_spot(F,:), color_F, G);
            hold on 
        end 
    end 
end 
pause(0.1);

N

%机器人真实位置
% P_pos_real = GetRandomPos(N, free_spot) 
P_pos_real = [5,6];
% P_pos_real = [5,69];
% P_pos_real = [46,30];
% P_pos_real = [59,23];
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
pos = GetPos(P_pos, MM);
p1 = plot(pos(1,:), pos(2,:),colorshape_p1,'markersize',markersize);
pos_R = GetPos(P_pos_real, MM);
pr = plot(pos_R(1,:), pos_R(2,:),colorshape_real,'MarkerFaceColor',color_real);

set(gca,'Layer','top');
pause(pause_time);

delete(p1);


%% Step2 PSO

ger1 = 30;% 初始位置预测pso迭代次数
real_scan_result = GetScanResult(G, P_pos_real);
real_fitness = CalFitness(real_scan_result, real_scan_result)

for iter1 = 1 : ger1
    % 获取适应度
    for i = 1:N
        P_scan_result = GetScanResult(G, P_pos(i,:));
        P_fitness(i) =  CalFitness(real_scan_result, P_scan_result);
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
                pdistance = ODistance(P_pos(i,:),Top_best_pos(j,:));
                if pdistance < P_min_distance(i)
                    L_best_pos(i,:) = Top_best_pos(j,:);
                    P_min_distance(i) = pdistance;
                end
        end
    end
    
    % 更新速度
    v = v * w + c_1 * rand *(P_best_pos - P_pos) + c_2 * rand *(L_best_pos - P_pos);
    v = edgeLimit(v, vlimit, d);% 边界位置处理
    

    % 更新位置
    P_pos_N = P_pos + v;
    P_pos_N = edgeLimit(P_pos_N, xlimit, d);% 边界位置处理
    
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
    pos = GetPos(P_pos, MM);
    p1 = plot(pos(1,:), pos(2,:),colorshape_p1,'markersize',markersize);
    pos_Top = GetPos(Top_best_pos, MM);
    p3 = plot(pos_Top(1,:), pos_Top(2,:),'d','Color',color_lbest,'MarkerFaceColor',color_lbest,'markersize',markersize2);
    pos_R = GetPos(P_pos_real, MM);
    pr = plot(pos_R(1,:), pos_R(2,:),colorshape_real,'MarkerFaceColor',color_real);
    
    pause(pause_time);
    delete(p1);
    delete(p3);
    %迭代次数
end

%% Step3 重采样
min_distance_threshold = 2;
for i = 1:N
    if P_min_distance(i) > min_distance_threshold
        P_pos(i,:) = L_best_pos(i,:);
        P_best_pos(i,:) = P_pos(i,:);
        P_scan_result = GetScanResult(G, P_pos(i,:));
        P_fitness(i) =  CalFitness(real_scan_result, P_scan_result);
        P_best_fitness(i) = P_fitness(i);
    end
end


% 显示
pos = GetPos(P_pos, MM);
p1 = plot(pos(1,:), pos(2,:),colorshape_resampling,'markersize',markersize);
pos_Top = GetPos(Top_best_pos, MM);
p3 = plot(pos_Top(1,:), pos_Top(2,:),'d','Color',color_lbest,'MarkerFaceColor',color_lbest,'markersize',markersize2);
pos_R = GetPos(P_pos_real, MM);
pr = plot(pos_R(1,:), pos_R(2,:),colorshape_real,'MarkerFaceColor',color_real);
pause(pause_time);
delete(p1);
delete(p3);

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
        neighbors=RegionQuery(i,cluster_distance,epsilon);
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
                    neighbors2=RegionQuery(j,cluster_distance,epsilon);
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
    pos = GetPos(s_P_pos, MM);
    p2 = plot(pos(1,:), pos(2,:),'*','Color',color_list_c(i,:),'markersize',markersize);
    
end
pos_Top = GetPos(Top_best_pos, MM);
p3 = plot(pos_Top(1,:), pos_Top(2,:),'d','Color',color_lbest,'MarkerFaceColor',color_lbest,'markersize',markersize2);
pos_R = GetPos(P_pos_real, MM);
pr = plot(pos_R(1,:), pos_R(2,:),colorshape_real,'MarkerFaceColor',color_real);

pause(pause_time);

delete(p3);
 

%% 姿态追踪
v_real = [4,0;0,4;0,4;0,6;4,0;0,6;0,4;4,1;5,0;3,5;4,-2;6,0;4,-1;5,1;5,-1;5,1;5,-1;3,1;4,-1];
% v_real = [4,-1;6,2;5,1;6,1;4,-2;6,0;5,-1;4,-2;0,-5;1,-4;3,-1;2,-3;-3,-5;-5,0;-3,-1;0,5;-1,6];
% v_real = [4,4;4,4;6,1;4,2;1,4;0,5;-1,5;0,4;1,5;1,6;2,3;2,3;-1,4;-4,-1;-4,-1];
% v_real = [4,1;4,2;0,4;1,4;1,4;0,5;-1,5;0,4;-5,0;-4,0;0,-4;1,-4;4,0];
% v_real = [0,4;2,4;1,1];
% v_real = [4,0;4,0;4,0;0,-4];
%% 移动
ger3 = 10;
dead_N = 0;
for iter3 =1 : ger3
    v_real(iter3,:)
    P_pos_real = P_pos_real + v_real(iter3,:)
    stepx = v_real(iter3,1)/abs(v_real(iter3,1));
    stepy = v_real(iter3,2)/abs(v_real(iter3,2));
    for i = 1:N 
        for x = 1:v_real(iter3,1)
            P_x = P_pos(i,1);
            P_x_round = round(P_x) + stepx; 
            P_y_round = round(P_pos(i,2));
            if P_x_round >= MM || P_x_round <= 1
                break;
            elseif G(P_x_round,P_y_round) == 1
                break;
            else
                P_pos(i,1) = P_x + stepx + rand(1,1)*0.5;
            end
        end
        
        for y = 1:v_real(iter3,2)
            P_y= P_pos(i,2) ;
            P_x_round = round(P_pos(i,1)); 
            P_y_round = round(P_y) + stepy;
            if P_y_round >= MM || P_y_round <= 1
                break;
            elseif G(P_x_round,P_y_round) == 1
                break;
            else
                P_pos(i,2) = P_y + stepy + rand(1,1)*0.5;
            end
        end
    end
    P_best_pos = P_pos;% 个体最优位置 
    P_best_fitness = -inf(N, 1);% 个体最优适应度
    % 显示新的真实位置
    cla reset;
    axis([0,MM,0,MM]) 
    W = 0; 
    F = 0; % 画图
    % 画图
    for i=1:MM
        for j=1:MM
            if G(i,j) == 1 %黑色代表占用
                %占用区域记录
                W = W + 1;
                wall_spot(W,1) = i;
                wall_spot(W,2) = j;
                % 画图
                SetColor(wall_spot(W,:), color_W, G);
                hold on
            else %白色代表空闲
                %空闲区域记录
                F = F + 1; 
                free_spot(F,1) = i;
                free_spot(F,2) = j;
                %P_pos(N,3)=1;%旋转角度 1-从正左，2-从左上，3-从正上，...，8-从左下
                % 画图
                SetColor(free_spot(F,:), color_F, G);
                hold on 
            end 
        end 
    end 
    
    
    min_distance_threshold = 2;
    %% 子群内部
    real_scan_result = GetScanResult(G, P_pos_real);
    real_fitness = CalFitness(real_scan_result, real_scan_result);
    subswarm_fitness = -inf(subswarm_N,1);
    for s = 1 : subswarm_N
        s
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
            s_P_scan_result = GetScanResult(G, s_P_pos(i,:));
            s_P_fitness(i) =  CalFitness(real_scan_result, s_P_scan_result);
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
            pdistance = ODistance(s_P_pos(i,:),s_G_best_pos);
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
        pos = GetPos(s_P_pos, MM);
        p2 =plot(pos(1,:), pos(2,:),'*','Color',color_list_c(s,:),'markersize',markersize);
        pos_g = GetPos(s_G_best_pos, MM);
        p3 = plot(pos_g(1,:), pos_g(2,:),'rd','MarkerFaceColor',color_list_c(s,:));
        pos_R = GetPos(P_pos_real, MM);
        pr = plot(pos_R(1,:), pos_R(2,:),colorshape_real,'MarkerFaceColor',color_real);
        
        subswarm_fitness(s) = mean(s_P_best_fitness);
        
    end

    % 画图
    cla reset;
    axis([0,MM,0,MM]) 
    W = 0; 
    F = 0; % 画图
    % 画图
    for i=1:MM
        for j=1:MM
            if G(i,j) == 1 %黑色代表占用
                %占用区域记录
                W = W + 1;
                wall_spot(W,1) = i;
                wall_spot(W,2) = j;
                % 画图
                SetColor(wall_spot(W,:), color_W, G);
                hold on
            else %白色代表空闲
                %空闲区域记录
                F = F + 1; 
                free_spot(F,1) = i;
                free_spot(F,2) = j;
                %P_pos(N,3)=1;%旋转角度 1-从正左，2-从左上，3-从正上，...，8-从左下
                % 画图
                SetColor(free_spot(F,:), color_F, G);
                hold on 
            end 
        end 
    end 

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
            neighbors=RegionQuery(i,cluster_distance,epsilon);
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
                        neighbors2=RegionQuery(j,cluster_distance,epsilon);
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
%         pos_D = GetPos(dead_P,MM);
%         pd = plot(pos_D(1,:), pos_D(2,:),'*','Color',[0.5,0.5,0.5],'markersize',markersize);
%     end
    for i = 1 : subswarm_N
        subswarm_current = subswarms(i,:);
        subswarm_current(subswarm_current==0)=[];
        s_P_pos = P_pos(subswarm_current,:);
        pos = GetPos(s_P_pos, MM);
        p2 = plot(pos(1,:), pos(2,:),'*','Color',color_list_c(i,:),'markersize',markersize);
    end
    pos_R = GetPos(P_pos_real, MM);
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

function SetColor (P_pos_xy, color, G)
i=P_pos_xy(1);
j=P_pos_xy(2);
MM=size(G,1);
x1=j-1;y1=MM-i; 
x2=j;y2=MM-i; 
x3=j;y3=MM-i+1; 
x4=j-1;y4=MM-i+1; 
fill([x1,x2,x3,x4],[y1,y2,y3,y4],color); 
end


function pos = GetPos (P_pos_xy, MM)
i=P_pos_xy(:,1)';
j=P_pos_xy(:,2)';
pos(1,:) = j - 0.5;
pos(2,:) = MM - i + 0.5;
end

function sim = CalFitness(disG_real, disG)
E = sum(disG_real.*disG_real,2);
F = sum(disG.*disG,2);
EF = sum(disG_real.*disG,2);
sim = EF / sqrt(E*F);
end

function scan_result = GetScanResult(G, P_pos_xy)
MM=size(G,1);
scan_result = zeros(1,8);
x = P_pos_xy(1);
y = P_pos_xy(2);

if x >= MM-1.5
    x = floor(x);
elseif x < 1.5
    x = ceil(x);
else
    x = round(x);
end
    

if y >= MM-1.5
    y = floor(y);
elseif y < 1.5
    y = ceil(y);
else 
    y = round(y);
end


for j = 0:MM-1 %左
    if G(x,y-j) == 1
        scan_result(1) = j-1;
        break;
    end
end

for j = 0:MM-1 %左上
  if G(x-j,y-j) == 1
        scan_result(2) = j-1;
        break;
    end
end

for j = 0:MM-1 %上
    if G(x-j,y) == 1
        scan_result(3) = j-1;
        break;
    end
end

for j = 0:MM %右上
    if G(x-j,y+j) == 1
        scan_result(4) = j-1;
        break;
    end
end
for j = 0:MM-1 %右
    if G(x,y+j) == 1
        scan_result(5) = j-1;
        break;
    end
end

for j = 0:MM %右下
    if G(x+j,y+j) == 1
        scan_result(6) = j-1;
        break;
    end
end
for j = 0:MM-1 %下
    if G(x+j,y) == 1
        scan_result(7) = j-1;
        break;
    end
end

for j = 0:MM-1 %左下
    if G(x+j,y-j) == 1
        scan_result(8)=j-1;
        break;
    end
end
end

function data = edgeLimit (data, limit, d)
N = size(data,1);
for i=1:d 
    for j=1:N
        if  data(j,i)>limit(i,2)
            data(j,i)=limit(i,2);
        end
        if  data(j,i) < limit(i,1)
            data(j,i)=limit(i,1);
        end
    end
end
end

function neighbors=RegionQuery(i,D,epsilon)
neighbors=find(D(i,:)<=epsilon);
end


function distance = ODistance (P_pos_1, P_pos_2)
x1 = P_pos_1(1);
x2 = P_pos_1(2);
y1 = P_pos_2(1);
y2 = P_pos_2(2);
distance = sqrt((y1-x1)*(y1-x1) +(y2-x2)*(y2-x2));
end

