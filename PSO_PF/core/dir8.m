function v = dir8(idx)
% 1�� 2���� 3�� 4���� 5�� 6���� 7�� 8����  ���������꣺x=�У�y=�У�
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
