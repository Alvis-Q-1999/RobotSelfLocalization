function sim = fitness(disG_real, disG)
% 计算两个扫描结果的相似度
E = sum(disG_real .* disG_real, 2);
F = sum(disG .* disG, 2);
EF = sum(disG_real .* disG, 2);
sim = EF / sqrt(E * F);
end
