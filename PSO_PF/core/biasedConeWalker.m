function step = biasedConeWalker(G, P_pos_real, lenRange, coneWidth, alpha)
% 在“最空旷方向”的邻域里做有偏随机游走（包含该方向在内的 coneWidth 个方向）
% 输入：
%   G            : 栅格地图（0=空地，1=障碍）
%   P_pos_real   : 当前真实位置 [i j]
%   lenRange     : 步长范围 [Lmin Lmax]（格数，默认 [1 6]）
%   coneWidth    : 邻域方向数量（含最空旷方向本身，默认 4）
%   alpha        : 权重指数，越大越偏好“更空旷”的方向（默认 1.5）
% 输出：
%   step         : 建议位移向量 [di dj]（整数）

    if nargin < 3 || isempty(lenRange),  lenRange  = [1 6];  end
    if nargin < 4 || isempty(coneWidth), coneWidth = 4;      end
    if nargin < 5 || isempty(alpha),     alpha     = 1.5;    end

    % 1) 扫描 8 个方向的可通行距离
    scan = scan8dirs(G, P_pos_real);             % 1..8
    [~, baseIdx] = max(scan);                    % 最空旷方向的索引（基准）

    % 2) 取以 baseIdx 为中心的 coneWidth 个候选方向（环形取邻居）
    %    例：coneWidth=4 → [base, base+1, base-1, base+2]（wrap 到 1..8）
    offsets = [0 1 -1 2];                        % 可按需更改（前向更重）
    offsets = offsets(1:coneWidth);
    candIdx = arrayfun(@(k) wrap8(baseIdx + k), offsets);

    % 3) 用扫描距离做加权（距离^alpha），再按权重随机抽一个方向
    w = max(scan(candIdx), 0) .^ alpha;
    if all(w==0), w = ones(size(w)); end
    w = w / sum(w);
    pick = randChoice(candIdx, w);

    % 4) 随机步长 & 生成位移（整数）
    L = randi(lenRange);
    step = L * dir8(pick);
    step = round(step);
    if all(step==0), step = dir8(baseIdx); end  % 兜底避免 0 步
end

% ======== 以下为本地小工具函数（只在本文件内可见） ========

function idx = wrap8(k)
% 把任意整数 k 映射到 1..8 的环形索引
    idx = mod(k-1, 8) + 1;
end

function v = dir8(idx)
% 1左 2左上 3上 4右上 5右 6右下 7下 8左下（行列坐标）
    switch idx
        case 1, v = [ 0, -1];
        case 2, v = [-1, -1];
        case 3, v = [-1,  0];
        case 4, v = [-1,  1];
        case 5, v = [ 0,  1];
        case 6, v = [ 1,  1];
        case 7, v = [ 1,  0];
        case 8, v = [ 1, -1];
        otherwise, v = [0,0];
    end
end

function pick = randChoice(values, probs)
% 按给定概率从 values 里随机选择一个
    edges = [0, cumsum(probs(:)')];
    r = rand;
    k = find(r >= edges(1:end-1) & r < edges(2:end), 1, 'first');
    pick = values(k);
end
