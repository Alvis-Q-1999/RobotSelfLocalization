function [dirIdx, dirVec] = chooseOpenDirection(G, P_pos_real)
% 用 scan8dirs 选距离障碍物最远的方向
scan = scan8dirs(G, P_pos_real);           % 1..8 方向距离
[~, dirIdx] = max(scan);                   % 下标就是方向号
dirVec = dir8(dirIdx);                     % 单位步向 [dx, dy]
end


