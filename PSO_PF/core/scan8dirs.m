function scan_result = scan8dirs(G, P_pos_xy)
% 等价于原 GetScanResult：在八个方向上扫描到障碍（1）前的距离
% 方向顺序与原代码一致：
% 1左、2左上、3上、4右上、5右、6右下、7下、8左下
MM = size(G,1);
scan_result = zeros(1,8);

x = P_pos_xy(1);
y = P_pos_xy(2);

% 与原代码完全一致的“边界取整”策略
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

% 左
for j = 0:MM-1
    if G(x, y-j) == 1
        scan_result(1) = j-1;
        break;
    end
end

% 左上
for j = 0:MM-1
    if G(x-j, y-j) == 1
        scan_result(2) = j-1;
        break;
    end
end

% 上
for j = 0:MM-1
    if G(x-j, y) == 1
        scan_result(3) = j-1;
        break;
    end
end

% 右上
for j = 0:MM
    if G(x-j, y+j) == 1
        scan_result(4) = j-1;
        break;
    end
end

% 右
for j = 0:MM-1
    if G(x, y+j) == 1
        scan_result(5) = j-1;
        break;
    end
end

% 右下
for j = 0:MM
    if G(x+j, y+j) == 1
        scan_result(6) = j-1;
        break;
    end
end

% 下
for j = 0:MM-1
    if G(x+j, y) == 1
        scan_result(7) = j-1;
        break;
    end
end

% 左下
for j = 0:MM-1
    if G(x+j, y-j) == 1
        scan_result(8) = j-1;
        break;
    end
end
end
