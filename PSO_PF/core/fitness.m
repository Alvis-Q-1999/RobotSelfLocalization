function sim = fitness(disG_real, disG)
% ��������ɨ���������ƶ�
E = sum(disG_real .* disG_real, 2);
F = sum(disG .* disG, 2);
EF = sum(disG_real .* disG, 2);
sim = EF / sqrt(E * F);
end
