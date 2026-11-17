function save_fig(fig, file)
% saves a figure to the results folder
p = params();
if ~exist(p.fig_dir, 'dir'), mkdir(p.fig_dir); end
print(fig, fullfile(p.fig_dir, [file '.png']), '-dpng', '-r200');
end
