function plotMap(G, color_W, color_F)

if nargin < 2, color_W = 'k'; end
if nargin < 3, color_F = 'w'; end

MM = size(G,1);
axis([0,MM,0,MM]); grid on; set(gca,'Layer','top'); hold on;

for i = 1:MM
    for j = 1:MM
        if G(i,j) == 1
            drawGridCell([i j], color_W, G);
        else
            drawGridCell([i j], color_F, G);
        end
    end
end
end
