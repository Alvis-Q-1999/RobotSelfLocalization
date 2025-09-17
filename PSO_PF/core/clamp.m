function data = clamp(data, limit, d)
% 等价于原 edgeLimit：把 data 的每一列裁剪到 limit 的区间内
% limit: d×2，每行 [min max]
N = size(data,1);
for i = 1:d
    for j = 1:N
        if data(j,i) > limit(i,2)
            data(j,i) = limit(i,2);
        end
        if data(j,i) < limit(i,1)
            data(j,i) = limit(i,1);
        end
    end
end
end
