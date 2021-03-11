function lineObst = get_line_obstacles()
%GET_LINE_OBSTACLES outputs an array of linear obstacles organized as such:
% col 1 : slope a
% col 2 : intercept b
% col 3 : x_start
% col 4 : x_end


numObst = 0;

lineObst = zeros(numObst,3);

for i = 1:numObst
   lineObst(i,1) = -1;
   lineObst(i,2) = 12;
   lineObst(i,3) = 3;
   lineObst(i,4) = 9;
end

end

