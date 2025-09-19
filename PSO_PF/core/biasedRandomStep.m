function step = biasedRandomStep(baseDir, lenRange, pTurn)
% baseDir: 1x2 ��λ�������� dir8��
% lenRange: [Lmin Lmax] ÿ�����ȣ�������
% pTurn: ��ƫ���ʣ�0~1������ 0.2 ��ʾ20%���ʲ�ƫ

if nargin<2, lenRange = [1 6]; end
if nargin<3, pTurn    = 0.2;  end

L = randi(lenRange);                  % �������
step = L * baseDir;                   % ���ػ�׼����

% �� pTurn �ĸ�����һ����΢��ƫ����1 �����ش�ֱ����
if rand < pTurn
    % ��һ���� baseDir ��ֱ�ķ���դ��������ƣ�
    ortho = [ -baseDir(2), baseDir(1) ];
    if ortho(1)==0 && ortho(2)==0      % ���� [0,0]
        ortho = [baseDir(2), -baseDir(1)];
    end
    ortho = sign(ortho);               % ѹ�� {-1,0,1}
    if all(ortho==0), ortho = [0 1]; end
    jitter = ortho * randi([1 5]);     % 0��1���ƫ
    if rand<0.5, jitter = -jitter; end
    step = step + jitter;
end

step = round(step);                    % ��֤����λ��
if all(step==0), step = baseDir; end   % ���� 0 ��
end
