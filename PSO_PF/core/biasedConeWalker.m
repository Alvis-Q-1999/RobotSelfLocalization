function step = biasedConeWalker(G, P_pos_real, lenRange, coneWidth, alpha)
% �ڡ���տ����򡱵�����������ƫ������ߣ������÷������ڵ� coneWidth ������
% ���룺
%   G            : դ���ͼ��0=�յأ�1=�ϰ���
%   P_pos_real   : ��ǰ��ʵλ�� [i j]
%   lenRange     : ������Χ [Lmin Lmax]��������Ĭ�� [1 6]��
%   coneWidth    : ����������������տ�������Ĭ�� 4��
%   alpha        : Ȩ��ָ����Խ��Խƫ�á����տ����ķ���Ĭ�� 1.5��
% �����
%   step         : ����λ������ [di dj]��������

    if nargin < 3 || isempty(lenRange),  lenRange  = [1 6];  end
    if nargin < 4 || isempty(coneWidth), coneWidth = 4;      end
    if nargin < 5 || isempty(alpha),     alpha     = 1.5;    end

    % 1) ɨ�� 8 ������Ŀ�ͨ�о���
    scan = scan8dirs(G, P_pos_real);             % 1..8
    [~, baseIdx] = max(scan);                    % ��տ��������������׼��

    % 2) ȡ�� baseIdx Ϊ���ĵ� coneWidth ����ѡ���򣨻���ȡ�ھӣ�
    %    ����coneWidth=4 �� [base, base+1, base-1, base+2]��wrap �� 1..8��
    offsets = [0 1 -1 2];                        % �ɰ�����ģ�ǰ����أ�
    offsets = offsets(1:coneWidth);
    candIdx = arrayfun(@(k) wrap8(baseIdx + k), offsets);

    % 3) ��ɨ���������Ȩ������^alpha�����ٰ�Ȩ�������һ������
    w = max(scan(candIdx), 0) .^ alpha;
    if all(w==0), w = ones(size(w)); end
    w = w / sum(w);
    pick = randChoice(candIdx, w);

    % 4) ������� & ����λ�ƣ�������
    L = randi(lenRange);
    step = L * dir8(pick);
    step = round(step);
    if all(step==0), step = dir8(baseIdx); end  % ���ױ��� 0 ��
end

% ======== ����Ϊ����С���ߺ�����ֻ�ڱ��ļ��ڿɼ��� ========

function idx = wrap8(k)
% ���������� k ӳ�䵽 1..8 �Ļ�������
    idx = mod(k-1, 8) + 1;
end

function v = dir8(idx)
% 1�� 2���� 3�� 4���� 5�� 6���� 7�� 8���£��������꣩
    switch idx
        case 1, v = [ 0, -1];
        case 2, v = [-1, -1];
        case 3, v = [-1,  0];
        case 4, v = [-1,  1];
        case 5, v = [ 0,  1];
        case 6, v = [ 1,  1];
        case 7, v = [ 1,  0];
        case 8, v = [ 1, -1];
        otherwise, v = [0,0];
    end
end

function pick = randChoice(values, probs)
% ���������ʴ� values �����ѡ��һ��
    edges = [0, cumsum(probs(:)')];
    r = rand;
    k = find(r >= edges(1:end-1) & r < edges(2:end), 1, 'first');
    pick = values(k);
end
