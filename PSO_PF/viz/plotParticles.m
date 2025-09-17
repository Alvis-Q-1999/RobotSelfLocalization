function [hParticles, hTop, hReal] = plotParticles(G, P_pos, Top_best_pos, P_pos_real, styles)
% 统一绘制：粒子、Top best、真实位置
% styles 结构体字段（可选，均有默认值）：
%   .colorshape_p1      默认 'm*'
%   .markersize         默认 4
%   .color_lbest        默认 [0.25 0.5 0.8]
%   .markersize2        默认 3
%   .colorshape_real    默认 'bd'
%   .color_real         默认 'b'

MM = size(G,1);

% 默认风格（与你原代码一致）
def.colorshape_p1   = 'm*';
def.markersize      = 4;
def.color_lbest     = [0.25, 0.5, 0.8];
def.markersize2     = 3;
def.colorshape_real = 'bd';
def.color_real      = 'b';

if nargin < 5 || isempty(styles), styles = struct(); end
fn = fieldnames(def);
for k = 1:numel(fn)
    if ~isfield(styles, fn{k})
        styles.(fn{k}) = def.(fn{k});
    end
end

% 粒子
hParticles = [];
if ~isempty(P_pos)
    pos = gridCoord(P_pos, MM);
    hParticles = plot(pos(1,:), pos(2,:), styles.colorshape_p1, 'markersize', styles.markersize);
    hold on;
end

% Top best
hTop = [];
if exist('Top_best_pos','var') && ~isempty(Top_best_pos)
    posTop = gridCoord(Top_best_pos, MM);
    hTop = plot(posTop(1,:), posTop(2,:), 'd', ...
        'Color', styles.color_lbest, 'MarkerFaceColor', styles.color_lbest, ...
        'markersize', styles.markersize2);
end

% 真实位置
hReal = [];
if exist('P_pos_real','var') && ~isempty(P_pos_real)
    posR = gridCoord(P_pos_real, MM);
    hReal = plot(posR(1,:), posR(2,:), styles.colorshape_real, 'MarkerFaceColor', styles.color_real);
end

set(gca, 'Layer', 'top');

end
