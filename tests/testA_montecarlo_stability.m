clear; clc; close all;
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src')));
%% Test A: Monte Carlo initial-condition sweep for stability evidence
% Pure Pursuit (outer loop) + 4WS allocation + wheel-speed PI + slip-speed tire
% Shows: bounded errors, convergence envelopes, success rate across random ICs
clear; clc; close all;

rng(11);

%% -------------------- Reference path (convincing) --------------------
% Path: straight -> S-bend -> straight (smooth, varying curvature)
% This is more convincing than only straight line.
pathXY = makePath_Scurve();

% quick plot
figure('Name','Reference Path'); hold on; axis equal; grid on;
plot(pathXY(:,1), pathXY(:,2), 'k--', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]'); title('Reference path (straight + S-bend + straight)');
hold off;

%% -------------------- Monte Carlo setup --------------------
Nruns = 60;                % 30~100 typical
Tmax  = 65;                % [s]
dt    = 0.02;
Nt    = floor(Tmax/dt)+1;

% Random initial condition ranges
posRange = 0.6;            % +/- [m] in both x,y around start
psiRange = deg2rad(70);    % +/- [rad] heading error range

startNom = pathXY(1,:);    % nominal start at first path point

% Storage
Ect_all   = nan(Nt, Nruns);   % cross-track error magnitude
Epsi_all  = nan(Nt, Nruns);   % heading error magnitude
succ      = false(Nruns,1);
tvec      = (0:Nt-1)'*dt;

% Save a few example trajectories
keepK = 10;
trajX = cell(keepK,1); trajY = cell(keepK,1);

%% -------------------- Vehicle & controller params (same spirit as yours) --------------------
P = struct();

% Vehicle
P.m   = 25;           % [kg]
P.Iz  = 2.0;          % [kg*m^2]
P.a   = 0.35;         % [m] CG->front axle
P.b   = 0.25;         % [m] CG->left wheel
P.Rw  = 0.06;         % [m]
P.Jw  = 0.02;         % [kg*m^2]
P.bw  = 0.03;         % wheel damping
P.mu  = 0.9;
P.g   = 9.81;
P.Nw  = P.m*P.g/4;

% Wheel positions (BODY frame)
P.Rpos = [ +P.a, +P.a, -P.a, -P.a;
           +P.b, -P.b, +P.b, -P.b];  % 2x4 (FL FR RL RR)

% Tire (slip-speed)
P.Cv  = 180;          % [N/(m/s)]

% Damping (helps stability / removes energy growth)
P.cV  = 0.7;
P.cR  = 0.5;

% Pure Pursuit outer loop
P.Ld   = 0.45;        % [m] lookahead
P.vMax = 1.0;         % [m/s]
P.wMax = 2.5;         % [rad/s]

% Wheel speed PI (inner loop)
P.Kp_w   = 0.6;
P.Ki_w   = 0.4;
P.tauMax = 2.5;
P.Imax   = 15;

% Termination conditions
P.snapRadius = 0.10;        % [m]
P.vStop      = 0.05;        % [m/s]
P.wStop      = 0.25;        % [rad/s]

%% -------------------- Run Monte Carlo --------------------
fprintf('Running %d Monte Carlo simulations...\n', Nruns);

for k = 1:Nruns
    % random initial position around nominal start
    x0 = startNom(1) + (2*rand()-1)*posRange;
    y0 = startNom(2) + (2*rand()-1)*posRange;

    % random initial heading around path tangent at start
    th0_path = atan2(pathXY(2,2)-pathXY(1,2), pathXY(2,1)-pathXY(1,1));
    psi0 = wrapToPi_local(th0_path + (2*rand()-1)*psiRange);

    % simulate
    out = simulateRun_PP_4WS(x0, y0, psi0, pathXY, P, dt, Tmax);

    % record errors
    nlen = numel(out.t);
    Ect_all(1:nlen, k)  = abs(out.e_ct);
    Epsi_all(1:nlen, k) = abs(out.e_psi);

    succ(k) = out.success;

    if k <= keepK
        trajX{k} = out.x;
        trajY{k} = out.y;
    end
end

fprintf('Done.\n');

%% -------------------- Plot: error envelopes (stability evidence) --------------------
% Compute envelopes ignoring NaNs
Ect_max  = max(Ect_all,  [], 2, 'omitnan');
Ect_mean = mean(Ect_all, 2, 'omitnan');

Epsi_max  = max(Epsi_all,  [], 2, 'omitnan');
Epsi_mean = mean(Epsi_all, 2, 'omitnan');

figure('Name','Stability Evidence: Error Envelopes'); grid on; hold on;
plot(tvec, Ect_max,  'LineWidth', 2);
plot(tvec, Ect_mean, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('|e_{ct}| [m]');
title(sprintf('Cross-track error envelopes (%d runs). Success rate = %.1f%%', Nruns, 100*mean(succ)));
legend('max |e_{ct}| over runs','mean |e_{ct}| over runs','Location','best');
hold off;

figure('Name','Stability Evidence: Heading Error Envelopes'); grid on; hold on;
plot(tvec, rad2deg(Epsi_max),  'LineWidth', 2);
plot(tvec, rad2deg(Epsi_mean), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('|e_{\psi}| [deg]');
title('Heading error envelopes');
legend('max |e_{\psi}| over runs','mean |e_{\psi}| over runs','Location','best');
hold off;

%% -------------------- Plot: representative trajectories --------------------
figure('Name','Representative Trajectories'); hold on; axis equal; grid on;
plot(pathXY(:,1), pathXY(:,2), 'k--', 'LineWidth', 2);
for k = 1:keepK
    if ~isempty(trajX{k})
        plot(trajX{k}, trajY{k}, 'LineWidth', 1.2);
    end
end
xlabel('X [m]'); ylabel('Y [m]');
title('Representative closed-loop trajectories (random ICs)');
legend('Reference path','Trajectories','Location','best');
hold off;

%% -------------------- Summary numbers (nice for PPT) --------------------
fprintf('\n===== Test A summary =====\n');
fprintf('Runs: %d\n', Nruns);
fprintf('Success rate: %.1f%%\n', 100*mean(succ));
fprintf('Peak max |e_ct| over time (worst case): %.3f m\n', max(Ect_max,[],'omitnan'));
fprintf('Final mean |e_ct| (last 2s): %.3f m\n', mean(Ect_mean(end-round(2/dt):end),'omitnan'));
fprintf('Final mean |e_psi| (last 2s): %.2f deg\n', mean(rad2deg(Epsi_mean(end-round(2/dt):end)),'omitnan'));

%% ===================== Local functions =====================

function pathXY = makePath_Scurve()
    % Double S path: (left->right) then (right->left), then straight
    % Smooth, varying curvature, good for long-run stability evidence.

    % Segment A: straight in x
    x1 = linspace(0, 4, 140);
    y1 = zeros(size(x1));

    % Segment B: S-bend #1 (left then right)
    x2 = linspace(4, 12, 360); x2 = x2(2:end);
    y2 = 1.2 * tanh(1.2*(x2-6.0)) - 1.2 * tanh(1.2*(x2-10.0));

    % Segment C: S-bend #2 (right then left) — reverse sign + shifted
    x3 = linspace(12, 20, 360); x3 = x3(2:end);
    y3shape = -(1.2 * tanh(1.2*(x3-14.0)) - 1.2 * tanh(1.2*(x3-18.0)));
    y3 = y2(end) + y3shape;   % connect continuously

    % Segment D: straight tail
    x4 = linspace(20, 24, 160); x4 = x4(2:end);
    y4 = y3(end) * ones(size(x4));

    % Combine
    x  = [x1 x2 x3 x4]';
    y  = [y1 y2 y3 y4]';

    % Rotate + shift (avoid trivial axis-aligned path)
    th = deg2rad(10);
    R  = [cos(th) -sin(th); sin(th) cos(th)];
    XY = (R*[x'; y'])';
    XY(:,1) = XY(:,1) + 2.0;
    XY(:,2) = XY(:,2) + 2.0;

    % Resample by arc length (robust) — uses your fixed resampleByArcLength
    pathXY = resampleByArcLength(XY, 0.04);
end


function XYr = resampleByArcLength(XY, ds)
    % Remove consecutive duplicate points (zero-length segments)
    dXY = diff(XY,1,1);
    segL = sqrt(sum(dXY.^2,2));
    keep = [true; segL > 1e-12];   % keep first + non-zero segments
    XY = XY(keep,:);

    % Recompute arc-length
    d  = sqrt(sum(diff(XY,1,1).^2,2));
    s  = [0; cumsum(d)];

    % If still not strictly increasing, enforce uniqueness (very robust)
    [s_u, ia] = unique(s, 'stable');
    XY_u = XY(ia,:);

    s2 = (0:ds:s_u(end))';
    x2 = interp1(s_u, XY_u(:,1), s2, 'linear');
    y2 = interp1(s_u, XY_u(:,2), s2, 'linear');
    XYr = [x2 y2];
end


function out = simulateRun_PP_4WS(x0, y0, psi0, pathXY, P, dt, Tmax)
    Nt = floor(Tmax/dt)+1;

    x   = x0;  y = y0;  psi = psi0;
    xd  = 0;   yd = 0;  psid = 0;

    omega_w = zeros(4,1);
    int_w   = zeros(4,1);

    % error logs
    tlog   = zeros(Nt,1);
    xlog   = zeros(Nt,1);
    ylog   = zeros(Nt,1);
    ectlog = zeros(Nt,1);
    epslog = zeros(Nt,1);

    lastIdx = 1;

    success = false;
    Npath = size(pathXY,1);

    for k = 1:Nt
        t = (k-1)*dt;

        % termination (near goal + near stop)
        distEnd = hypot(x - pathXY(end,1), y - pathXY(end,2));
        if distEnd < P.snapRadius && hypot(xd,yd) < P.vStop && abs(psid) < P.wStop
            success = true;
            tlog   = tlog(1:k);
            xlog   = xlog(1:k);
            ylog   = ylog(1:k);
            ectlog = ectlog(1:k);
            epslog = epslog(1:k);
            break;
        end

        % ===== errors to path (for plotting stability) =====
        [idxNear, e_ct, th_path] = crossTrackError([x y], pathXY);
        e_psi = wrapToPi_local(th_path - psi);

        % ===== reference for Pure Pursuit =====
        [xref, yref, ~, lastIdx] = getRefOnPathForward([x y], pathXY, P.Ld, lastIdx);

        % near end lock to goal
        if lastIdx >= Npath-3
            xref = pathXY(end,1);
            yref = pathXY(end,2);
        end

        dxg = xref - x;
        dyg = yref - y;

        alpha = wrapToPi_local(atan2(dyg, dxg) - psi);
        ePos  = hypot(dxg, dyg);

        % speed schedule
        v_des = min(P.vMax, 0.9 * ePos);

        % Pure Pursuit curvature -> yaw rate
        kappa_pp = 2*sin(alpha) / max(P.Ld, 1e-3);
        w_des = v_des * kappa_pp;
        w_des = max(min(w_des, P.wMax), -P.wMax);

        % slow down when turning hard
        v_des = min(v_des, P.vMax/(1 + 0.9*abs(w_des)));

        % desired body velocities
        vx_des = v_des; vy_des = 0; om_des = w_des;

        % current body velocity from world velocity
        Rwb = [ cos(psi)  sin(psi);
               -sin(psi)  cos(psi) ];
        v_body = Rwb * [xd; yd];
        vx = v_body(1); vy = v_body(2); om = psid;

        % wheel steering + desired wheel ground speed
        delta = zeros(4,1);
        vcmd  = zeros(4,1);
        vlong = zeros(4,1);

        for i = 1:4
            xi = P.Rpos(1,i); yi = P.Rpos(2,i);

            % desired contact velocity at wheel (body frame)
            vix_des = vx_des - om_des*yi;
            viy_des = vy_des + om_des*xi;

            delta(i) = atan2(viy_des, vix_des);
            vcmd(i)  = hypot(vix_des, viy_des);

            % actual contact velocity at wheel (body frame)
            vix = vx - om*yi;
            viy = vy + om*xi;

            h = [cos(delta(i)); sin(delta(i))];
            vlong(i) = h.' * [vix; viy];
        end

        omega_des = vcmd / P.Rw;

        % inner loop: PI torque -> wheel omega; slip-speed tire -> Fx
        F   = zeros(4,1);
        tau = zeros(4,1);

        for i = 1:4
            ew = omega_des(i) - omega_w(i);
            int_w(i) = int_w(i) + ew*dt;
            int_w(i) = max(min(int_w(i), P.Imax), -P.Imax);

            tau(i) = P.Kp_w*ew + P.Ki_w*int_w(i);
            tau(i) = max(min(tau(i), P.tauMax), -P.tauMax);

            s = P.Rw*omega_w(i) - vlong(i);      % slip speed
            F(i) = P.Cv * s;
            F(i) = max(min(F(i),  P.mu*P.Nw), -P.mu*P.Nw);

            omega_wdot = (tau(i) - P.Rw*F(i) - P.bw*omega_w(i)) / P.Jw;
            omega_w(i) = omega_w(i) + omega_wdot*dt;
        end

        % EOM in WORLD frame: qdd = M^{-1} B F
        B = zeros(3,4);
        for i = 1:4
            B(1,i) = cos(psi + delta(i));
            B(2,i) = sin(psi + delta(i));
            B(3,i) = P.Rpos(1,i)*sin(delta(i)) - P.Rpos(2,i)*cos(delta(i));
        end

        qdd = [ (B(1,:)*F)/P.m;
                (B(2,:)*F)/P.m;
                (B(3,:)*F)/P.Iz ];

        % damping
        qdd(1) = qdd(1) - P.cV*xd;
        qdd(2) = qdd(2) - P.cV*yd;
        qdd(3) = qdd(3) - P.cR*psid;

        % integrate
        xd   = xd   + qdd(1)*dt;
        yd   = yd   + qdd(2)*dt;
        psid = psid + qdd(3)*dt;

        x    = x    + xd*dt;
        y    = y    + yd*dt;
        psi  = wrapToPi_local(psi + psid*dt);

        % log
        tlog(k)   = t;
        xlog(k)   = x;
        ylog(k)   = y;
        ectlog(k) = e_ct;
        epslog(k) = e_psi;
    end

    out.t = tlog;
    out.x = xlog;
    out.y = ylog;
    out.e_ct = ectlog;
    out.e_psi = epslog;
    out.success = success;
end

function [idxNear, e_ct, th_path] = crossTrackError(pos, pathXY)
    % Nearest point on polyline (vertex-based) + signed cross-track error to local segment
    % For presentation: vertex-nearest is ok; we also estimate tangent heading.
    dif = pathXY - pos;
    [~, idxNear] = min(sum(dif.^2,2));

    % tangent estimate
    N = size(pathXY,1);
    i0 = max(1, min(N-1, idxNear));
    dp = pathXY(i0+1,:) - pathXY(i0,:);
    th_path = atan2(dp(2), dp(1));

    % signed cross-track (2D): sign from left/right of tangent
    v = [cos(th_path), sin(th_path)];
    n = [-v(2), v(1)];
    e_ct = n * (pos(:) - pathXY(i0,:).');
end

function [xref, yref, thref, lastIdx] = getRefOnPathForward(pos, pathXY, Ld, lastIdx)
    N = size(pathXY,1);

    searchStart = max(1, lastIdx - 10);
    seg = pathXY(searchStart:N,:) - pos;
    d2  = sum(seg.^2, 2);
    [~, k] = min(d2);
    idx = searchStart + k - 1;

    idx = max(idx, lastIdx);
    lastIdx = idx;

    j = idx;
    while j < N && hypot(pathXY(j,1)-pos(1), pathXY(j,2)-pos(2)) < Ld
        j = j + 1;
    end
    j = min(j, N);

    xref = pathXY(j,1);
    yref = pathXY(j,2);

    if j < N
        dp = pathXY(j+1,:) - pathXY(j,:);
    else
        dp = pathXY(j,:) - pathXY(max(j-1,1),:);
    end
    thref = atan2(dp(2), dp(1));
end

function ang = wrapToPi_local(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end

