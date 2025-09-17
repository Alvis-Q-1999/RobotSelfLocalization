function pos = gridCoord(P_pos_xy, MM)
% 等价于原 GetPos(P_pos_xy, MM)

i = P_pos_xy(:,1)';
j = P_pos_xy(:,2)';

pos(1,:) = j - 0.5;
pos(2,:) = MM - i + 0.5;
end
