function scan_result = scan8dirs(G, P_pos_xy)
% 等价于原 GetScanResult：在八个方向上扫描到障碍（1）或边界前的距离
% 方向顺序：1左、2左上、3上、4右上、5右、6右下、7下、8左下
MM = size(G,1);
scan_result = zeros(1,8);

x = P_pos_xy(1);
y = P_pos_xy(2);

% 与原代码一致的“边界取整”策略
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

% 8 个方向的单位增量（行、列）
dirs = [ 0 -1;   % 1 左
        -1 -1;   % 2 左上
        -1  0;   % 3 上
        -1  1;   % 4 右上
         0  1;   % 5 右
         1  1;   % 6 右下
         1  0;   % 7 下
         1 -1];  % 8 左下

% 统一的安全扫描：遇到边界或障碍就停，并返回 j-1
for k = 1:8
    dx = dirs(k,1);
    dy = dirs(k,2);
    for j = 1:MM
        nx = x + dx*j;
        ny = y + dy*j;

        % 先做边界检查：到边界即停，距离为 j-1
        if nx < 1 || nx > MM || ny < 1 || ny > MM
            scan_result(k) = j - 1;
            break;
        end

        % 碰到障碍：距离为 j-1
        if G(nx, ny) == 1
            scan_result(k) = j - 1;
            break;
        end
    end
end
end
