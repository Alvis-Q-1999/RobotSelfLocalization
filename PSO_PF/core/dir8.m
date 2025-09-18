function v = dir8(idx)
% 1左 2左上 3上 4右上 5右 6右下 7下 8左下  （行列坐标：x=行，y=列）
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
