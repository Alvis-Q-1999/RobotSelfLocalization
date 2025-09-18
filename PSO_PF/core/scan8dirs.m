function scan_result = scan8dirs(G, P_pos_xy)
% �ȼ���ԭ GetScanResult���ڰ˸�������ɨ�赽�ϰ���1����߽�ǰ�ľ���
% ����˳��1��2���ϡ�3�ϡ�4���ϡ�5�ҡ�6���¡�7�¡�8����
MM = size(G,1);
scan_result = zeros(1,8);

x = P_pos_xy(1);
y = P_pos_xy(2);

% ��ԭ����һ�µġ��߽�ȡ��������
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

% 8 ������ĵ�λ�������С��У�
dirs = [ 0 -1;   % 1 ��
        -1 -1;   % 2 ����
        -1  0;   % 3 ��
        -1  1;   % 4 ����
         0  1;   % 5 ��
         1  1;   % 6 ����
         1  0;   % 7 ��
         1 -1];  % 8 ����

% ͳһ�İ�ȫɨ�裺�����߽���ϰ���ͣ�������� j-1
for k = 1:8
    dx = dirs(k,1);
    dy = dirs(k,2);
    for j = 1:MM
        nx = x + dx*j;
        ny = y + dy*j;

        % �����߽��飺���߽缴ͣ������Ϊ j-1
        if nx < 1 || nx > MM || ny < 1 || ny > MM
            scan_result(k) = j - 1;
            break;
        end

        % �����ϰ�������Ϊ j-1
        if G(nx, ny) == 1
            scan_result(k) = j - 1;
            break;
        end
    end
end
end
