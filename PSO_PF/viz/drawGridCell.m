function drawGridCell(P_pos_xy, color, G)

i = P_pos_xy(1);
j = P_pos_xy(2);
MM = size(G,1);

x1 = j-1; y1 = MM - i;
x2 = j;   y2 = MM - i;
x3 = j;   y3 = MM - i + 1;
x4 = j-1; y4 = MM - i + 1;

fill([x1,x2,x3,x4],[y1,y2,y3,y4],color);
end
