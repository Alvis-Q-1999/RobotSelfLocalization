function [dirIdx, dirVec] = chooseOpenDirection(G, P_pos_real)
% �� scan8dirs ѡ�����ϰ�����Զ�ķ���
scan = scan8dirs(G, P_pos_real);           % 1..8 �������
[~, dirIdx] = max(scan);                   % �±���Ƿ����
dirVec = dir8(dirIdx);                     % ��λ���� [dx, dy]
end


