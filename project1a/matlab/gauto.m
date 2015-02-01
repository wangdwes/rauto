
varname = ['X', 'Y', 'Z', 'R', 'P', 'Y'];
plotpos = [1 2 3 4 5 6]; 
vars = [e_pos; e_att];

for idx = 1: 6
  subplot (2, 3, plotpos(idx)); hold on; 
  set (gca, 'XLim', [0 err_idx * 0.05]); 
  line ([0 samples], [0 0], 'LineStyle', '--', 'Color', 'r', 'LineWidth', 1.0); 
  plot (e_time(1:err_idx-1), vars(idx, 1:err_idx-1), 'Color', 'b', 'LineWidth', 1.0);
  title ([varname(idx) ' Error']);
  grid on; hold off; 
end

set (gcf, 'PaperPositionMode', 'auto');

% kpz = 2.5896; % z-propotional 
% kdz = 1.0869; % z-differential 
% desz = 0.5;   % desired z

% function gauto(Kp, Kd, pos_des)

% run the test using the assigned gains. 
% run dostuff

% then plot a graph showing how z changes. 
% close all;
% end

%{

figure ('position', [300 300 600 150]);
hold on;
set (gca, 'XLim', [0 samples*0.6], 'YLim', [0 desz+desz]);
set (gca, 'XTickLabel', [], 'YTickLabel', []);
set (gcf, 'PaperPositionMode', 'auto')
line ([0 samples], [desz desz], 'LineStyle', '--', 'Color', 'r', 'LineWidth', 1.0);
plot (pos_odom(3, :), 'Color', 'b', 'LineWidth', 1.0);
legend ('DES', [num2str(kpz) ',' num2str(kdz)]);
grid on;
hold off;

%}

% fname = ['~/rauto/project1a/plots/' num2str(kpz) '-' num2str(kdz) '.eps']; 
% print('-depsc', fname)

% end 
