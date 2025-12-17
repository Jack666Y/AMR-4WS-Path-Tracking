clear; clc; close all;
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src')));
%% Test A: Monte Carlo initial-condition sweep
% Pure Pursuit + 4WS allocation + wheel PI + slip-speed tire model
clear; clc; close all;

rng(11);

%% -------------------- Reference path --------------------
% Straight + S-bend + straight
pathXY = makePath_Scurve();

figure('Name','Reference Path'); hold on; axis equal; grid on;
plot(pathXY(:,1), pathXY(:,2), 'k--', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
title('Reference path');
hold off;

%% -------------------- Monte Carlo setup --------------------
Nruns = 60;
Tmax  = 65;
dt    = 0.02;
Nt    = floor(Tmax/dt)+1;

% Initial condition ranges
posRange = 0.6;                 % [m]
psiRange = deg2rad(70);         % [rad]
startNom = pathXY(1,:);

% Storage
Ect_all  = nan(Nt, Nruns);
Epsi_all = nan(Nt, Nruns);
succ     = false(Nruns,1);
tvec     = (0:Nt-1)'*dt;

% Save representative trajectories
keepK = 10;
trajX = cell(keepK,1);
trajY = cell(keepK,1);

%% -------------------- Vehicle & controller parameters --------------------
P = struct();

% Vehicle parameters
P.m  = 25;
P.Iz = 2.0;
P.a  = 0.35;
P.b  = 0.25;
P.Rw = 0.06;
P.Jw = 0.02;
P.bw = 0.03;
P.mu = 0.9;
P.g  = 9.81;
P.Nw = P.m*P.g/4;

% Wheel positions (body frame)
P.Rpos = [ +P.a, +P.a, -P.a, -P.a;
           +P.b, -P.b, +P.b, -P.b ];

% Tire and damping
P.Cv = 180;
P.cV = 0.7;
P.cR = 0.5;

% Pure Pursuit
P.Ld   = 0.45;
P.vMax = 1.0;
P.wMax = 2.5;

% Wheel speed PI
P.Kp_w   = 0.6;
P.Ki_w   = 0.4;
P.tauMax = 2.5;
P.Imax   = 15;

% Termination thresholds
P.snapRadius = 0.10;
P.vStop      = 0.05;
P.wStop      = 0.25;

%% -------------------- Monte Carlo simulations --------------------
fprintf('Running %d Monte Carlo simulations...\n', Nruns);

for k = 1:Nruns
    % Random initial state
    x0 = startNom(1) + (2*rand()-1)*posRange;
    y0 = startNom(2) + (2*rand()-1)*posRange;

    th0_path = atan2(pathXY(2,2)-pathXY(1,2), ...
                     pathXY(2,1)-pathXY(1,1));
    psi0 = wrapToPi_local(th0_path + (2*rand()-1)*psiRange);

    % Simulate
    out = simulateRun_PP_4WS(x0, y0, psi0, pathXY, P, dt, Tmax);

    % Record errors
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

%% -------------------- Error envelopes --------------------
Ect_max  = max(Ect_all,  [], 2, 'omitnan');
Ect_mean = mean(Ect_all, 2, 'omitnan');
Epsi_max  = max(Epsi_all,  [], 2, 'omitnan');
Epsi_mean = mean(Epsi_all, 2, 'omitnan');

figure; hold on; grid on;
plot(tvec, Ect_max,  'LineWidth', 2);
plot(tvec, Ect_mean, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('|e_{ct}| [m]');
title(sprintf('Cross-track error (%d runs)', Nruns));
legend('max','mean');
hold off;

figure; hold on; grid on;
plot(tvec, rad2deg(Epsi_max),  'LineWidth', 2);
plot(tvec, rad2deg(Epsi_mean), 'LineWidth', 2);
xlabel('Time [s]'); ylabel('|e_{\psi}| [deg]');
title('Heading error');
legend('max','mean');
hold off;

%% -------------------- Representative trajectories --------------------
figure; hold on; axis equal; grid on;
plot(pathXY(:,1), pathXY(:,2), 'k--', 'LineWidth', 2);
for k = 1:keepK
    if ~isempty(trajX{k})
        plot(trajX{k}, trajY{k});
    end
end
xlabel('X [m]'); ylabel('Y [m]');
title('Representative trajectories');
legend('Reference','Trajectories');
hold off;

%% -------------------- Summary --------------------
fprintf('\n===== Test A summary =====\n');
fprintf('Runs: %d\n', Nruns);
fprintf('Success rate: %.1f%%\n', 100*mean(succ));
fprintf('Worst |e_ct|: %.3f m\n', max(Ect_max,[],'omitnan'));
fprintf('Final mean |e_ct|: %.3f m\n', mean(Ect_mean(end-round(2/dt):end),'omitnan'));
fprintf('Final mean |e_psi|: %.2f deg\n', ...
        mean(rad2deg(Epsi_mean(end-round(2/dt):end)),'omitnan'));

%% ===================== Local functions =====================
function out = simulateRun_PP_4WS(x0, y0, psi0, pathXY, P, dt, Tmax)
    Nt = floor(Tmax/dt)+1;

    x   = x0;  y = y0;  psi = psi0;
    xd  = 0;   yd = 0;  psid = 0;

    omega_w = zeros(4,1);
    int_w   = zeros(4,1);

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

        [idxNear, e_ct, th_path] = crossTrackError([x y], pathXY);
        e_psi = wrapToPi_local(th_path - psi);

        [xref, yref, ~, lastIdx] = getRefOnPathForward([x y], pathXY, P.Ld, lastIdx);

        if lastIdx >= Npath-3
            xref = pathXY(end,1);
            yref = pathXY(end,2);
        end

        dxg = xref - x;
        dyg = yref - y;

        alpha = wrapToPi_local(atan2(dyg, dxg) - psi);
        ePos  = hypot(dxg, dyg);

        v_des = min(P.vMax, 0.9 * ePos);

        kappa_pp = 2*sin(alpha) / max(P.Ld, 1e-3);
        w_des = v_des * kappa_pp;
        w_des = max(min(w_des, P.wMax), -P.wMax);

        v_des = min(v_des, P.vMax/(1 + 0.9*abs(w_des)));

        vx_des = v_des; vy_des = 0; om_des = w_des;

        Rwb = [ cos(psi)  sin(psi);
               -sin(psi)  cos(psi) ];
        v_body = Rwb * [xd; yd];
        vx = v_body(1); vy = v_body(2); om = psid;

        delta = zeros(4,1);
        vcmd  = zeros(4,1);
        vlong = zeros(4,1);

        for i = 1:4
            xi = P.Rpos(1,i); yi = P.Rpos(2,i);

            vix_des = vx_des - om_des*yi;
            viy_des = vy_des + om_des*xi;

            delta(i) = atan2(viy_des, vix_des);
            vcmd(i)  = hypot(vix_des, viy_des);

            vix = vx - om*yi;
            viy = vy + om*xi;

            h = [cos(delta(i)); sin(delta(i))];
            vlong(i) = h.' * [vix; viy];
        end

        omega_des = vcmd / P.Rw;

        F   = zeros(4,1);
        tau = zeros(4,1);

        for i = 1:4
            ew = omega_des(i) - omega_w(i);
            int_w(i) = int_w(i) + ew*dt;
            int_w(i) = max(min(int_w(i), P.Imax), -P.Imax);

            tau(i) = P.Kp_w*ew + P.Ki_w*int_w(i);
            tau(i) = max(min(tau(i), P.tauMax), -P.tauMax);

            s = P.Rw*omega_w(i) - vlong(i);     
            F(i) = P.Cv * s;
            F(i) = max(min(F(i),  P.mu*P.Nw), -P.mu*P.Nw);

            omega_wdot = (tau(i) - P.Rw*F(i) - P.bw*omega_w(i)) / P.Jw;
            omega_w(i) = omega_w(i) + omega_wdot*dt;
        end

        B = zeros(3,4);
        for i = 1:4
            B(1,i) = cos(psi + delta(i));
            B(2,i) = sin(psi + delta(i));
            B(3,i) = P.Rpos(1,i)*sin(delta(i)) - P.Rpos(2,i)*cos(delta(i));
        end

        qdd = [ (B(1,:)*F)/P.m;
                (B(2,:)*F)/P.m;
                (B(3,:)*F)/P.Iz ];


        qdd(1) = qdd(1) - P.cV*xd;
        qdd(2) = qdd(2) - P.cV*yd;
        qdd(3) = qdd(3) - P.cR*psid;

        xd   = xd   + qdd(1)*dt;
        yd   = yd   + qdd(2)*dt;
        psid = psid + qdd(3)*dt;

        x    = x    + xd*dt;
        y    = y    + yd*dt;
        psi  = wrapToPi_local(psi + psid*dt);

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
