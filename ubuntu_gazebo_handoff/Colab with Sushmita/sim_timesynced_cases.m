% Time-synced MATLAB simulation of the real 4-robot experiments (Cases 1b-4).
%
% Exp_Collision_single_inte.m integrates the single-integrator law with ode45,
% whose time axis is not real seconds. The real robots are unicycles driven by
% multi_unicycle_controller_double_angle_experiment_ekf.py, which applies the
% same law through a look-ahead conversion with saturation at 50 Hz. This
% script replicates that loop, starts each run from the logged initial poses,
% and samples the result at the real CSV timestamps so every sim row lines up
% 1:1 with the corresponding real row.
%
% Two simulations per case, both from the logged initial positions and both
% sampled at the real CSV timestamps (sim t = 0 at the first real row):
%   single integrator : p_dot = u(p), ode45, same law/gain as Exp_Collision_single_inte.m,
%                       each robot's speed capped at P.si_speed_cap (= real v_max)
%   unicycle          : the real 50 Hz loop (look-ahead conversion + saturation + stop)
%
% Outputs (in ./sim_timesynced/), same columns as the real log_data.csv:
%   <case>_sim_single_integrator.csv   (theta = direction of motion)
%   <case>_sim_unicycle.csv
%   <case>_real_vs_sim.png             real (solid), single integrator (dashed), unicycle (dotted)

clear; close all;

figDir  = fullfile(fileparts(mfilename('fullpath')), '..', 'APS Report', 'Figures');
outDir  = fullfile(fileparts(mfilename('fullpath')), 'sim_timesynced');
if ~exist(outDir, 'dir'); mkdir(outDir); end

% Case name, collision avoidance on/off (from the report, Chapter 3)
cases = {'Case-1b', false;
         'Case-2',  true;
         'Case-3',  false;
         'Case-4',  true};

P = controller_params();

for k = 1:size(cases, 1)
    name = cases{k, 1}; useCA = cases{k, 2};
    real = readtable(fullfile(figDir, name, 'log_data.csv'));
    t_real = real.time;

    % Initial pose = first logged row (x, y, theta per bot)
    x0 = zeros(12, 1);
    for b = 0:3
        x0(3*b+1) = real.(sprintf('bot_%d_x', b))(1);
        x0(3*b+2) = real.(sprintf('bot_%d_y', b))(1);
        x0(3*b+3) = real.(sprintf('bot_%d_theta', b))(1);
    end

    Xsi  = simulate_single_integrator(x0, t_real, useCA, P);
    Xuni = simulate_unicycles(x0, t_real, useCA, P);

    si  = build_log_table(t_real, Xsi, P);
    uni = build_log_table(t_real, Xuni, P);
    writetable(si,  fullfile(outDir, [name '_sim_single_integrator.csv']));
    writetable(uni, fullfile(outDir, [name '_sim_unicycle.csv']));
    plot_real_vs_sim(real, si, uni, name, useCA, P, fullfile(outDir, [name '_real_vs_sim.png']));

    maxE = @(T) max(abs(table2array(T(end, {'e1','e2','e3','e4','e5'}))));
    fprintf('%s (CA %s): %d rows, %.1f s, final max|e| real = %.3f, single int = %.3f, unicycle = %.3f\n', ...
        name, mat2str(useCA), height(si), t_real(end) - t_real(1), maxE(real), maxE(si), maxE(uni));
end


% ======================================================================
function P = controller_params()
    % Values from multi_unicycle_controller_double_angle_experiment_ekf.py
    P.pstar = [0.4;0.3;  -0.4;0.3;  -0.7;-0.4;  0.7;-0.4];
    H_Nset = [-1 1 0 0; 0 -1 1 0; 0 0 -1 1; 1 0 0 -1; 0 1 0 -1];
    P.Hbar_Nset = kron(H_Nset, eye(2));
    P.Hbar_Mset = kron([-1 0 1 0], eye(2));

    P.r = 0.135; P.Rs = 0.16; P.Rc = 1.6; P.Rcrit = 0.4;
    P.v_max = 0.055; P.omega_max = 0.13; P.length = 0.25;
    P.si_speed_cap = P.v_max;                     % single-integrator per-robot speed cap (Inf = uncapped)
    P.conv_tol = 0.080;
    P.dt = 1/50;                                  % control loop rate

    Rc_bar = 1 - 2*(P.r/(P.Rc - P.r))^2;
    Rs_bar = 1 - 2*(P.r/(P.Rs + P.r))^2;
    Rcrit_bar = 1 - 2*(P.r/P.Rcrit)^2;
    P.psi_crit = acosd(Rcrit_bar);
    P.delta = Rs_bar^2 - Rcrit_bar^2;

    estar = P.Hbar_Nset*P.pstar;
    P.dstar = zeros(5,1); P.psi_star = zeros(5,1); P.Kc = zeros(5,1); P.Ks = zeros(5,1);
    for c = 1:5
        P.dstar(c) = norm(estar(2*c-1:2*c));
        P.psi_star(c) = 2*asind(P.r/P.dstar(c));
        P.Kc(c) = Rc_bar^2 - cosd(P.psi_star(c))^2;
        P.Ks(c) = cosd(P.psi_star(c))^2 - Rs_bar^2;
    end
end


% ======================================================================
function X = simulate_single_integrator(x0, t_out, useCA, P)
    % p_dot = u(p) with ode45 on the real time axis, as in
    % Exp_Collision_single_inte.m, except each robot's velocity is scaled
    % down (direction kept) so its speed never exceeds P.si_speed_cap.
    % No convergence stop.
    % theta is filled with the direction of motion (held when stationary).
    t_rel = t_out - t_out(1);
    p0 = x0([1 2 4 5 7 8 10 11]);
    opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
    [~, Pout] = ode45(@(t, p) capped_velocity(p, useCA, P), t_rel, p0, opts);
    X = zeros(numel(t_rel), 12);
    th = x0([3 6 9 12])';
    for j = 1:numel(t_rel)
        u = capped_velocity(Pout(j,:)', useCA, P);
        for i = 1:4
            if norm(u(2*i-1:2*i)) > 1e-6; th(i) = atan2(u(2*i), u(2*i-1)); end
        end
        X(j, [1 2 4 5 7 8 10 11]) = Pout(j,:);
        X(j, [3 6 9 12]) = th;
    end
end


function u = capped_velocity(p, useCA, P)
    u = formation_velocity(p, useCA, P);
    for i = 1:4
        s = norm(u(2*i-1:2*i));
        if s > P.si_speed_cap; u(2*i-1:2*i) = u(2*i-1:2*i)*(P.si_speed_cap/s); end
    end
end


% ======================================================================
function X = simulate_unicycles(x0, t_out, useCA, P)
    % Fixed-step 50 Hz loop: compute (v, w) from the current state, hold it for
    % one control period, and integrate the unicycle kinematics exactly.
    % Sim t = 0 is pinned to the first real CSV row; the state is returned
    % at each real timestamp.
    t_rel = t_out - t_out(1);
    nSteps = ceil(t_rel(end)/P.dt) + 1;
    T = (0:nSteps)'*P.dt;
    S = zeros(nSteps+1, 12); S(1,:) = x0';
    s = x0;
    for n = 1:nSteps
        [v, w] = control_law(s, useCA, P);
        for i = 0:3
            th = s(3*i+3);
            if abs(w(i+1)) > 1e-9
                s(3*i+1) = s(3*i+1) + v(i+1)/w(i+1)*(sin(th + w(i+1)*P.dt) - sin(th));
                s(3*i+2) = s(3*i+2) - v(i+1)/w(i+1)*(cos(th + w(i+1)*P.dt) - cos(th));
            else
                s(3*i+1) = s(3*i+1) + v(i+1)*cos(th)*P.dt;
                s(3*i+2) = s(3*i+2) + v(i+1)*sin(th)*P.dt;
            end
            s(3*i+3) = atan2(sin(th + w(i+1)*P.dt), cos(th + w(i+1)*P.dt));
        end
        S(n+1,:) = s';
    end
    % Sample at real timestamps (interpolate positions; heading via sin/cos)
    X = zeros(numel(t_rel), 12);
    for i = 0:3
        X(:,3*i+1) = interp1(T, S(:,3*i+1), t_rel);
        X(:,3*i+2) = interp1(T, S(:,3*i+2), t_rel);
        X(:,3*i+3) = atan2(interp1(T, sin(S(:,3*i+3)), t_rel), interp1(T, cos(S(:,3*i+3)), t_rel));
    end
end


% ======================================================================
function [v, w] = control_law(s, useCA, P)
    % Real-robot loop: convergence stop, then look-ahead unicycle conversion
    p = s([1 2 4 5 7 8 10 11]);
    theta = s([3 6 9 12]);
    v = zeros(4,1); w = zeros(4,1);

    edge_err = zeros(5,1);
    e_N = P.Hbar_Nset*p;
    for c = 1:5; edge_err(c) = norm(e_N(2*c-1:2*c)) - P.dstar(c); end
    if all(abs(edge_err) <= P.conv_tol); return; end

    d_dt = formation_velocity(p, useCA, P);

    for i = 1:4
        ux = d_dt(2*i-1); uy = d_dt(2*i);
        v(i) = 2*(ux*cos(theta(i)) + uy*sin(theta(i)));
        w(i) = (-ux*sin(theta(i)) + uy*cos(theta(i)))/P.length;
    end
    v = max(min(v, P.v_max), -P.v_max);
    w = max(min(w, P.omega_max), -P.omega_max);
end


function d_dt = formation_velocity(p, useCA, P)
    % Single-integrator law of Exp_Collision_single_inte.m
    d_dt = zeros(8,1);
    e_N = P.Hbar_Nset*p;
    e_M = P.Hbar_Mset*p;

    R_psi_N = []; xi = zeros(5,1);
    for c = 1:5
        eij = e_N(2*c-1:2*c);
        dij = norm(eij);
        if dij < 1e-6; return; end
        psi = 2*asind(min(P.r/dij, 1));
        R_psi_N = blkdiag(R_psi_N, bearing_block(eij/dij, psi));
        E = cosd(psi)^2 - cosd(P.psi_star(c))^2;
        if E > 0; K = P.Kc(c); else; K = P.Ks(c); end
        den = K^2 - E^2;
        if abs(den) >= 1e-6; xi(c) = 4*E/den; end
    end

    d_dt = -(P.Hbar_Nset'*R_psi_N*xi);

    if useCA
        eik = e_M(1:2);
        dik = norm(eik);
        if dik < 1e-6; return; end
        psi = 2*asind(min(P.r/dik, 1));
        R_psi_M = bearing_block(eik/dik, psi);
        if psi > P.psi_crit
            beta = cosd(psi)^2 - cosd(P.psi_crit)^2;
        else
            beta = 0;
        end
        den = P.delta^2 - beta^2;
        xi_beta = 0;
        if abs(den) >= 1e-6; xi_beta = 4*beta/den; end
        d_dt = d_dt - P.Hbar_Mset'*R_psi_M*xi_beta;
    end
end


function B = bearing_block(g, psi)
    Rot_R = [cosd(psi/2) -sind(psi/2); sind(psi/2) cosd(psi/2)];
    Rot_L = [cosd(psi/2) sind(psi/2); -sind(psi/2) cosd(psi/2)];
    gR = Rot_R*g; gL = Rot_L*g;
    f_g = (eye(2) - gL*gL')*gR + (eye(2) - gR*gR')*gL;
    f_psi = 0.5*sind(2*psi)/(1 + cosd(psi));
    B = f_psi*f_g;
end


% ======================================================================
function T = build_log_table(t, X, P)
    n = numel(t);
    T = table(t, 'VariableNames', {'time'});
    for b = 0:3
        T.(sprintf('bot_%d_x', b))     = X(:,3*b+1);
        T.(sprintf('bot_%d_y', b))     = X(:,3*b+2);
        T.(sprintf('bot_%d_theta', b)) = X(:,3*b+3);
    end
    D = zeros(n, 5);
    for j = 1:n
        e = P.Hbar_Nset*X(j, [1 2 4 5 7 8 10 11])';
        for c = 1:5; D(j,c) = norm(e(2*c-1:2*c)); end
    end
    for c = 1:5; T.(sprintf('d%d', c)) = D(:,c); end
    for c = 1:5; T.(sprintf('e%d', c)) = D(:,c) - P.dstar(c); end
    T.lower_bound = repmat(P.Rs + P.r, n, 1);
    T.upper_bound = repmat(P.Rc - P.r, n, 1);
end


% ======================================================================
function plot_real_vs_sim(real, si, uni, name, useCA, P, outFile)
    pairs = [0 1; 1 2; 2 3; 3 0; 1 3; 0 2];       % 5 graph edges + collision pair
    cols = lines(6);
    botCols = [0 0 1; 0 0.6 0; 1 0 0; 0 0.75 0.75];
    srcs = {real, '-', 'real'; si, '--', 'single int'; uni, ':', 'unicycle'};

    fig = figure('Visible', 'off', 'Position', [0 0 1500 600]);
    tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
    if useCA; caStr = 'on'; else; caStr = 'off'; end
    title(tl, sprintf('%s (collision avoidance %s): real (solid), single integrator (dashed), unicycle (dotted)', name, caStr));

    ax1 = nexttile(tl); hold(ax1, 'on'); axis(ax1, 'equal'); grid(ax1, 'on');
    for b = 0:3
        for k = 1:3
            T = srcs{k,1};
            x = T.(sprintf('bot_%d_x', b)); y = T.(sprintf('bot_%d_y', b));
            plot(ax1, x, y, srcs{k,2}, 'Color', botCols(b+1,:), 'LineWidth', 1.2, ...
                'DisplayName', sprintf('bot\\_%d %s', b, srcs{k,3}));
            plot(ax1, x(end), y(end), 'o', 'Color', botCols(b+1,:), 'MarkerFaceColor', botCols(b+1,:), 'HandleVisibility', 'off');
        end
        plot(ax1, real.(sprintf('bot_%d_x', b))(1), real.(sprintf('bot_%d_y', b))(1), 'o', ...
            'Color', botCols(b+1,:), 'MarkerSize', 9, 'HandleVisibility', 'off');
    end
    xlabel(ax1, 'x (m)'); ylabel(ax1, 'y (m)');
    legend(ax1, 'Location', 'bestoutside');

    ax2 = nexttile(tl); hold(ax2, 'on'); grid(ax2, 'on');
    for k = 1:6
        a = pairs(k,1); b = pairs(k,2);
        for m = 1:3
            T = srcs{m,1};
            d = hypot(T.(sprintf('bot_%d_x', a)) - T.(sprintf('bot_%d_x', b)), ...
                      T.(sprintf('bot_%d_y', a)) - T.(sprintf('bot_%d_y', b)));
            plot(ax2, T.time, d, srcs{m,2}, 'Color', cols(k,:), 'LineWidth', 1.2, ...
                'DisplayName', sprintf('d_{%d%d} %s', a, b, srcs{m,3}));
        end
    end
    yline(ax2, P.Rs + P.r, 'k:', 'R_s + r', 'HandleVisibility', 'off');
    yline(ax2, P.Rc - P.r, 'k:', 'R_c - r', 'HandleVisibility', 'off');
    xlabel(ax2, 'time (s)'); ylabel(ax2, 'distance (m)');
    legend(ax2, 'Location', 'bestoutside', 'NumColumns', 3);

    exportgraphics(fig, outFile, 'Resolution', 150);
    close(fig);
end
