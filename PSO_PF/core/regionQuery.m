function neighbors = regionQuery(i, D, epsilon)
% 等价于原 regionQuery：DBSCAN 的邻域点查询
% D 为全体点对距离矩阵，返回距离 <= epsilon 的索引
neighbors = find(D(i,:) <= epsilon);
end
