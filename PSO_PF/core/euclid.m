function distance = euclid(P_pos_1, P_pos_2)
% �ȼ���ԭ ODistance����άŷ����þ���
x1 = P_pos_1(1); x2 = P_pos_1(2);
y1 = P_pos_2(1); y2 = P_pos_2(2);
distance = sqrt((y1 - x1) * (y1 - x1) + (y2 - x2) * (y2 - x2));
end
