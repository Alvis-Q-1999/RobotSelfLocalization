function [P_pos, free_spot, wall_spot, N] = initParticles(G)
%INITPARTICLES 从二值地图 G 中提取粒子初始位置与空闲/障碍格坐标
%   输入:
%     G:  二值地图矩阵 (0=空地, 1=障碍)，必须为方阵
%   输出:
%     P_pos(N,2):     粒子初始位置（所有空地格的 [i j]）
%     free_spot(F,2): 空地格坐标列表（与 P_pos 一致）
%     wall_spot(W,2): 障碍格坐标列表
%     N:              粒子数（也就是空地格数）
%
%   说明:
%   - 为保证与原 main_final.m 的行为一致，本函数按 i=1..MM, j=1..MM 的
%     次序遍历并填充数组（与原始双层 for 循环顺序完全相同）。
%   - 不进行任何绘图，绘图请在 viz/ 中完成（如 plotMap）。

    % 基本检查
    [rows, cols] = size(G);
    if rows ~= cols
        error('initParticles:MapMustBeSquare', ...
              'G 必须是方阵，但当前尺寸为 %dx%d。', rows, cols);
    end
    MM = rows;

    % 预估大小（提高效率）：空地数 F 与障碍数 W
    F_est = nnz(G == 0);
    W_est = nnz(G == 1);

    % 预分配
    P_pos     = zeros(F_est, 2);
    free_spot = zeros(F_est, 2);
    wall_spot = zeros(W_est, 2);

    % 计数器
    N = 0;   % 粒子数（= 空地数）
    F = 0;   % 空地计数
    W = 0;   % 障碍计数

    % 遍历地图（与原代码相同的顺序与逻辑）
    for i = 1:MM
        for j = 1:MM
            if G(i,j) == 1
                W = W + 1;
                wall_spot(W, :) = [i, j];
            else
                N = N + 1;
                P_pos(N, :)     = [i, j];
                F = F + 1;
                free_spot(F, :) = [i, j];
            end
        end
    end

    % （安全起见）如果实际数量 < 预估，截断多余的预分配行
    if F < F_est
        P_pos(F+1:end, :)     = [];
        free_spot(F+1:end, :) = [];
    end
    if W < W_est
        wall_spot(W+1:end, :) = [];
    end
end
