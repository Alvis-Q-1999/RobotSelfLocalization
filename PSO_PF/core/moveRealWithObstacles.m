function P_next = moveRealWithObstacles(G, P_now, v)
% 按 v=[dx,dy] 推进真实位置，逐轴移动、遇墙/边界即停
P_next = P_now;
MM = size(G,1);

% x 轴（行）方向
sx = sign(v(1)); ax = abs(v(1));
for k = 1:ax
    cand = P_next; cand(1) = cand(1) + sx;
    if cand(1) < 1 || cand(1) > MM, break; end
    if G(round(cand(1)), round(cand(2))) == 1, break; end
    P_next(1) = cand(1);
end

% y 轴（列）方向
sy = sign(v(2)); ay = abs(v(2));
for k = 1:ay
    cand = P_next; cand(2) = cand(2) + sy;
    if cand(2) < 1 || cand(2) > MM, break; end
    if G(round(cand(1)), round(cand(2))) == 1, break; end
    P_next(2) = cand(2);
end
end


