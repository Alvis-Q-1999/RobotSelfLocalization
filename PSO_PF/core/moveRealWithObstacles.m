function P_next = moveRealWithObstacles(G, P_now, v)
% �� v=[dx,dy] �ƽ���ʵλ�ã������ƶ�����ǽ/�߽缴ͣ
P_next = P_now;
MM = size(G,1);

% x �ᣨ�У�����
sx = sign(v(1)); ax = abs(v(1));
for k = 1:ax
    cand = P_next; cand(1) = cand(1) + sx;
    if cand(1) < 1 || cand(1) > MM, break; end
    if G(round(cand(1)), round(cand(2))) == 1, break; end
    P_next(1) = cand(1);
end

% y �ᣨ�У�����
sy = sign(v(2)); ay = abs(v(2));
for k = 1:ay
    cand = P_next; cand(2) = cand(2) + sy;
    if cand(2) < 1 || cand(2) > MM, break; end
    if G(round(cand(1)), round(cand(2))) == 1, break; end
    P_next(2) = cand(2);
end
end


