function step = biasedRandomStep(baseDir, lenRange, pTurn)
% baseDir: 1x2 单位方向（来自 dir8）
% lenRange: [Lmin Lmax] 每步长度（格数）
% pTurn: 侧偏概率（0~1），如 0.2 表示20%概率侧偏

if nargin<2, lenRange = [1 6]; end
if nargin<3, pTurn    = 0.2;  end

L = randi(lenRange);                  % 随机步长
step = L * baseDir;                   % 先沿基准方向

% 以 pTurn 的概率做一次轻微侧偏（±1 个格沿垂直方向）
if rand < pTurn
    % 求一条与 baseDir 垂直的方向（栅格内最近似）
    ortho = [ -baseDir(2), baseDir(1) ];
    if ortho(1)==0 && ortho(2)==0      % 避免 [0,0]
        ortho = [baseDir(2), -baseDir(1)];
    end
    ortho = sign(ortho);               % 压到 {-1,0,1}
    if all(ortho==0), ortho = [0 1]; end
    jitter = ortho * randi([1 5]);     % 0或1格侧偏
    if rand<0.5, jitter = -jitter; end
    step = step + jitter;
end

step = round(step);                    % 保证整数位移
if all(step==0), step = baseDir; end   % 避免 0 步
end
