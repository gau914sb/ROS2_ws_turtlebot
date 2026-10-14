% Time-synced distance-error plots, real vs both simulations, for Cases 1b-4.
% Reads the CSVs written by sim_timesynced_cases.m (run that first).
%
% e_k = d_k - d_k^*, for the five connectivity edges (same columns in all CSVs).
% Outputs (in ./sim_timesynced/):
%   <case>_distance_errors.png/.pdf   one panel per edge
%   all_cases_distance_errors.png/.pdf  5 edges x 4 cases grid

clear; close all;

here   = fileparts(mfilename('fullpath'));
figDir = fullfile(here, '..', 'APS Report', 'Figures');
outDir = fullfile(here, 'sim_timesynced');

cases  = {'Case-1b', 'off'; 'Case-2', 'on'; 'Case-3', 'off'; 'Case-4', 'on'};
edges  = {'e_1  (bot_0-bot_1)', 'e_2  (bot_1-bot_2)', 'e_3  (bot_2-bot_3)', ...
          'e_4  (bot_3-bot_0)', 'e_5  (bot_1-bot_3)'};
conv_tol = 0.080;                                 % robot convergence tolerance (m)

% Fixed source order: colour + line style both encode the source
src.name  = {'Real', 'Single-integrator sim', 'Unicycle sim'};
src.color = {hex2rgb('#2a78d6'), hex2rgb('#eb6834'), hex2rgb('#1baf7a')};
src.style = {'-', '--', ':'};
src.width = [1.0, 1.6, 1.8];

data = cell(size(cases, 1), 3);
for k = 1:size(cases, 1)
    name = cases{k, 1};
    data{k, 1} = readtable(fullfile(figDir, name, 'log_data.csv'));
    data{k, 2} = readtable(fullfile(outDir, [name '_sim_single_integrator.csv']));
    data{k, 3} = readtable(fullfile(outDir, [name '_sim_unicycle.csv']));
end

% ---------------- one figure per case ----------------
for k = 1:size(cases, 1)
    fig = figure('Visible', 'off', 'Position', [0 0 900 1050], 'Color', 'w');
    tl = tiledlayout(fig, 5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('%s (collision avoidance %s): distance error e_k = d_k - d_k^*', ...
        cases{k, 1}, cases{k, 2}), 'FontSize', 13);
    for c = 1:5
        ax = nexttile(tl);
        h = draw_edge(ax, data(k, :), c, src, conv_tol);
        ylabel(ax, edges{c}, 'FontSize', 10);
        if c < 5; ax.XTickLabel = []; end
        if c == 1; legend(ax, h, src.name, 'Orientation', 'horizontal', 'Location', 'northoutside', 'Box', 'off'); end
    end
    xlabel(tl, 'time (s)');
    save_fig(fig, fullfile(outDir, [cases{k, 1} '_distance_errors']));
end

% ---------------- combined grid ----------------
fig = figure('Visible', 'off', 'Position', [0 0 1800 1100], 'Color', 'w');
tl = tiledlayout(fig, 5, 4, 'TileSpacing', 'compact', 'Padding', 'compact');
title(tl, 'Distance error e_k = d_k - d_k^*: real vs single-integrator vs unicycle simulation (time-synced)', 'FontSize', 14);
for c = 1:5
    for k = 1:size(cases, 1)
        ax = nexttile(tl);
        h = draw_edge(ax, data(k, :), c, src, conv_tol);
        if c == 1; title(ax, sprintf('%s (CA %s)', cases{k, 1}, cases{k, 2}), 'FontSize', 11); end
        if k == 1; ylabel(ax, edges{c}, 'FontSize', 10); end
        if c == 5; xlabel(ax, 'time (s)'); end
        if c == 1 && k == 1
            lg = legend(ax, h, src.name, 'Orientation', 'horizontal', 'Box', 'off');
            lg.Layout.Tile = 'north';
        end
    end
end
save_fig(fig, fullfile(outDir, 'all_cases_distance_errors'));

fprintf('Wrote distance-error plots to %s\n', outDir);


% ======================================================================
function h = draw_edge(ax, D, c, src, conv_tol)
    hold(ax, 'on'); box(ax, 'off');
    col = sprintf('e%d', c);
    t_end = D{1}.time(end);
    patch(ax, [0 t_end t_end 0], [-conv_tol -conv_tol conv_tol conv_tol], [0.92 0.92 0.92], ...
        'EdgeColor', 'none', 'HandleVisibility', 'off');
    yline(ax, 0, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 0.5, 'HandleVisibility', 'off');
    h = gobjects(3, 1);
    for m = 1:3
        h(m) = plot(ax, D{m}.time, D{m}.(col), src.style{m}, 'Color', src.color{m}, 'LineWidth', src.width(m));
    end
    uistack(h(1), 'bottom'); uistack(h(1), 'up', 2);   % real under the sims, above the band
    xlim(ax, [0 t_end]);
    ax.YGrid = 'on'; ax.GridColor = [0.85 0.85 0.85]; ax.GridAlpha = 1;
    ax.XColor = [0.35 0.35 0.35]; ax.YColor = [0.35 0.35 0.35];
end


function save_fig(fig, base)
    exportgraphics(fig, [base '.png'], 'Resolution', 200);
    exportgraphics(fig, [base '.pdf'], 'ContentType', 'vector');
    close(fig);
end


function rgb = hex2rgb(h)
    rgb = sscanf(h(2:end), '%2x%2x%2x', [1 3])/255;
end
