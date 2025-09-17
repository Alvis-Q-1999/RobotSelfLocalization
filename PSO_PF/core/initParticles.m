function [P_pos, free_spot, wall_spot, N] = initParticles(G)
%INITPARTICLES �Ӷ�ֵ��ͼ G ����ȡ���ӳ�ʼλ�������/�ϰ�������
%   ����:
%     G:  ��ֵ��ͼ���� (0=�յ�, 1=�ϰ�)������Ϊ����
%   ���:
%     P_pos(N,2):     ���ӳ�ʼλ�ã����пյظ�� [i j]��
%     free_spot(F,2): �յظ������б��� P_pos һ�£�
%     wall_spot(W,2): �ϰ��������б�
%     N:              ��������Ҳ���ǿյظ�����
%
%   ˵��:
%   - Ϊ��֤��ԭ main_final.m ����Ϊһ�£��������� i=1..MM, j=1..MM ��
%     ���������������飨��ԭʼ˫�� for ѭ��˳����ȫ��ͬ����
%   - �������κλ�ͼ����ͼ���� viz/ ����ɣ��� plotMap����

    % �������
    [rows, cols] = size(G);
    if rows ~= cols
        error('initParticles:MapMustBeSquare', ...
              'G �����Ƿ��󣬵���ǰ�ߴ�Ϊ %dx%d��', rows, cols);
    end
    MM = rows;

    % Ԥ����С�����Ч�ʣ����յ��� F ���ϰ��� W
    F_est = nnz(G == 0);
    W_est = nnz(G == 1);

    % Ԥ����
    P_pos     = zeros(F_est, 2);
    free_spot = zeros(F_est, 2);
    wall_spot = zeros(W_est, 2);

    % ������
    N = 0;   % ��������= �յ�����
    F = 0;   % �յؼ���
    W = 0;   % �ϰ�����

    % ������ͼ����ԭ������ͬ��˳�����߼���
    for i = 1:MM
        for j = 1:MM
            if G(i,j) == 1
                W = W + 1;
                wall_spot(W, :) = [i, j];
            else
                N = N + 1;
                P_pos(N, :)     = [i, j];
                F = F + 1;
                free_spot(F, :) = [i, j];
            end
        end
    end

    % ����ȫ��������ʵ������ < Ԥ�����ض϶����Ԥ������
    if F < F_est
        P_pos(F+1:end, :)     = [];
        free_spot(F+1:end, :) = [];
    end
    if W < W_est
        wall_spot(W+1:end, :) = [];
    end
end
