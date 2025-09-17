function scan_result = scan8dirs(G, P_pos_xy)
% �ȼ���ԭ GetScanResult���ڰ˸�������ɨ�赽�ϰ���1��ǰ�ľ���
% ����˳����ԭ����һ�£�
% 1��2���ϡ�3�ϡ�4���ϡ�5�ҡ�6���¡�7�¡�8����
MM = size(G,1);
scan_result = zeros(1,8);

x = P_pos_xy(1);
y = P_pos_xy(2);

% ��ԭ������ȫһ�µġ��߽�ȡ��������
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

% ��
for j = 0:MM-1
    if G(x, y-j) == 1
        scan_result(1) = j-1;
        break;
    end
end

% ����
for j = 0:MM-1
    if G(x-j, y-j) == 1
        scan_result(2) = j-1;
        break;
    end
end

% ��
for j = 0:MM-1
    if G(x-j, y) == 1
        scan_result(3) = j-1;
        break;
    end
end

% ����
for j = 0:MM
    if G(x-j, y+j) == 1
        scan_result(4) = j-1;
        break;
    end
end

% ��
for j = 0:MM-1
    if G(x, y+j) == 1
        scan_result(5) = j-1;
        break;
    end
end

% ����
for j = 0:MM
    if G(x+j, y+j) == 1
        scan_result(6) = j-1;
        break;
    end
end

% ��
for j = 0:MM-1
    if G(x+j, y) == 1
        scan_result(7) = j-1;
        break;
    end
end

% ����
for j = 0:MM-1
    if G(x+j, y-j) == 1
        scan_result(8) = j-1;
        break;
    end
end
end
