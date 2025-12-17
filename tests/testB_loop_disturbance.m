clear; clc; close all;
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src')));
%% FINAL: Closed-loop tracking on oval track with disturbances
% Pure Pursuit + 4WS allocation + wheel PI + slip-speed tire + rigid-body dynamics
clear; clc; close all;
rng(11);

%% -------------------- Simulation --------------------
dt   = 0.02;
Tmax = 170;
Nt   = floor(Tmax/dt)+1;
tvec = (0:Nt-1)'*dt;

%% -------------------- Reference: closed loop path --------------------
pathXY = makePath_OvalTrack();
pathXY = resampleByArcLength(pathXY, 0.04);
Path   = preprocessPathClosed(pathXY);

figure('Name','Reference loop'); hold on; axis equal; grid on;
plot(pathXY(:,1), pathXY(:,2), 'k--', 'LineWidth', 2);
xlabel('X [m]'); ylabel('Y [m]');
title('Reference oval loop');
hold off;

%% -------------------- Parameters --------------------
P = struct();

% Vehicle
P.m   = 25;
P.Iz  = 2.0;
P.a   = 0.35;
P.b   = 0.25;
P.Rw  = 0.06;
P.Jw  = 0.02;
P.bw  = 0.03;

P.g   = 9.81;
P.mu0 = 0.9;
P.mu  = P.mu0;
P.Nw  = P.m*P.g/4;

% Wheel positions (body frame): FL FR RL RR
P.Rpos = [ +P.a, +P.a, -P.a, -P.a;
           +P.b, -P.b, +P.b, -P.b ];

% Tire / damping
P.Cv  = 180;
P.cV  = 0.7;
P.cR  = 0.5;

% Pure Pursuit
P.Ld   = 0.3;
P.vMax = 0.9;
P.wMax = 2.8;

% Wheel speed PI
P.Kp_w   = 0.6;
P.Ki_w   = 0.4;
P.tauMax = 2.5;
P.Imax   = 15;

% Start-up assistance
P.alphaMax = deg2rad(75);
P.v_min    = 0.25;
P.tAlign   = 2.0;

%% -------------------- Disturbances (time-triggered) --------------------
D = struct();
D.tEvent   = [40, 80];                 % event times [s]
D.nEvents  = numel(D.tEvent);
D.used     = false(D.nEvents,1);

% Event type: 1 lateral impulse, 2 yaw-rate impulse, 3 friction drop
D.type = [1, 1];

% Magnitudes
D.Fy_impulse_Ns  = 25;                 % [N*s]
D.r_impulse_rads = 2;                  % [rad/s]
D.mu_low         = 0.3;
D.mu_duration    = 10;

mu_hist    = P.mu0*ones(Nt,1);
evtMark    = false(Nt,1);
evtTypeLog = zeros(Nt,1);

%% -------------------- Initial state --------------------
x = pathXY(1,1) + 0.3;
y = pathXY(1,2) - 0.3;

dp0 = pathXY(2,:) - pathXY(1,:);
psi = atan2(dp0(2), dp0(1)) + deg2rad(20);

xd = 0; yd = 0; psid = 0;

omega_w = zeros(4,1);
int_w   = zeros(4,1);

% Arc-length progress state (loop-safe reference)
sLast = projectToPathS([x y], Path);

%% -------------------- Logs --------------------
xlog  = zeros(Nt,1); ylog  = zeros(Nt,1);
ect   = zeros(Nt,1); epsih = zeros(Nt,1);
vlog  = zeros(Nt,1); wlog  = zeros(Nt,1);

%% -------------------- Live animation --------------------
figLive = figure('Name','LIVE tracking'); clf; hold on; axis equal; grid on;
plot(pathXY(:,1), pathXY(:,2), 'k--', 'LineWidth', 2);
hTraj  = animatedline('Color','b','LineWidth',1.8);
hRobot = plot(x, y, 'bo', 'MarkerSize',6, 'LineWidth',2);
hEvt   = plot(nan,nan,'ro','MarkerSize',8,'LineWidth',1.8);

xlabel('X [m]'); ylabel('Y [m]');
title('LIVE: Reference vs Actual');
legend('Reference path','Actual trajectory','Robot','Event','Location','best');
drawnow;

%% -------------------- Main loop --------------------
for k = 1:Nt
    t = (k-1)*dt;

    % --- Apply disturbances at scheduled times ---
    for ei = 1:D.nEvents
        if ~D.used(ei) && t >= D.tEvent(ei)
            D.used(ei) = true;
            evtMark(k) = true;

            pick = D.type(ei);
            evtTypeLog(k) = pick;

            if pick == 1
                % lateral impulse along path normal (world frame)
                J = (2*rand()-1) * D.Fy_impulse_Ns;

                % NOTE: th_path is updated below each step
                nx = -sin(th_path);
                ny =  cos(th_path);

                xd = xd + (J/P.m) * nx;
                yd = yd + (J/P.m) * ny;

            elseif pick == 2
                % yaw-rate impulse
                psid = psid + (2*rand()-1)*D.r_impulse_rads;

            else
                % friction drop for duration
                D.muDropUntil = t + D.mu_duration;
            end
        end
    end

    % friction schedule
    if isfield(D,'muDropUntil') && t <= D.muDropUntil
        P.mu = D.mu_low;
    else
        P.mu = P.mu0;
    end
    mu_hist(k) = P.mu;

    % --- Errors to path (nearest segment) ---
    [~, e_ct_k, th_path] = crossTrackError([x y], pathXY);
    e_psi_k = wrapToPi_local(th_path - psi);

    % --- Loop-safe lookahead reference via arc-length s ---
    [xref, yref, thref, sLast] = getRefOnPathForwardClosed([x y], Path, P.Ld, sLast);

    % --- Pure Pursuit outer loop ---
    dxg = xref - x;  dyg = yref - y;
    alpha = wrapToPi_local(atan2(dyg, dxg) - psi);
    alpha = max(min(alpha, P.alphaMax), -P.alphaMax);

    ePos  = hypot(dxg, dyg);
    v_des = min(P.vMax, 0.9*ePos);

    kappa = 2*sin(alpha) / max(P.Ld, 1e-3);
    w_des = max(min(v_des*kappa, P.wMax), -P.wMax);

    v_des = min(v_des, P.vMax/(1 + 0.9*abs(w_des)));
    if ePos > 0.4
        v_des = max(v_des, P.v_min);
    end

    % start alignment mode
    if t < P.tAlign
        ePsi0 = wrapToPi_local(thref - psi);
        w_des = max(min(2.0*ePsi0, P.wMax), -P.wMax);
        v_des = 0.25;
    end

    vx_des = v_des; vy_des = 0; om_des = w_des;

    % --- Current body velocity ---
    Rwb = [ cos(psi)  sin(psi);
           -sin(psi)  cos(psi) ];
    v_body = Rwb * [xd; yd];
    vx = v_body(1); vy = v_body(2); om = psid;

    % --- 4WS allocation (steering + desired wheel speed) ---
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

    % --- Wheel PI + slip-speed tire ---
    F = zeros(4,1);
    for i = 1:4
        ew = omega_des(i) - omega_w(i);
        int_w(i) = max(min(int_w(i) + ew*dt, P.Imax), -P.Imax);

        tau = P.Kp_w*ew + P.Ki_w*int_w(i);
        tau = max(min(tau, P.tauMax), -P.tauMax);

        s = P.Rw*omega_w(i) - vlong(i);
        F(i) = P.Cv * s;

        Fmax = P.mu * P.Nw;
        F(i) = max(min(F(i), Fmax), -Fmax);

        omega_wdot = (tau - P.Rw*F(i) - P.bw*omega_w(i)) / P.Jw;
        omega_w(i) = omega_w(i) + omega_wdot*dt;
    end

    % --- Rigid-body dynamics in world frame ---
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

    % integrate
    xd   = xd   + qdd(1)*dt;
    yd   = yd   + qdd(2)*dt;
    psid = psid + qdd(3)*dt;

    x    = x    + xd*dt;
    y    = y    + yd*dt;
    psi  = wrapToPi_local(psi + psid*dt);

    % log
    xlog(k)=x; ylog(k)=y;
    ect(k)=e_ct_k; epsih(k)=e_psi_k;
    vlog(k)=hypot(xd,yd); wlog(k)=psid;

    % --- live plot update ---
    addpoints(hTraj, x, y);
    set(hRobot, 'XData', x, 'YData', y);
    if evtMark(k)
        set(hEvt, 'XData', x, 'YData', y);
    end

    plotStride = 4;
    if mod(k, plotStride) == 0
        drawnow limitrate;
        pause(dt);
    end
end

%% -------------------- Plots --------------------
figure('Name','Loop tracking'); hold on; axis equal; grid on;
plot(pathXY(:,1), pathXY(:,2), 'k--', 'LineWidth', 2);
plot(xlog, ylog, 'b-', 'LineWidth', 1.8);
idxEvt = find(evtMark);
plot(xlog(idxEvt), ylog(idxEvt), 'ro', 'MarkerSize', 10, 'LineWidth', 1.5);
xlabel('X [m]'); ylabel('Y [m]');
title('Closed-loop tracking with disturbances');
legend('Reference','Actual','Events','Location','best');

figure('Name','Errors'); tiledlayout(3,1,'Padding','compact','TileSpacing','compact');

nexttile; hold on; grid on;
plot(tvec, abs(ect), 'LineWidth', 1.6);
ylabel('|e_{ct}| [m]'); title('Errors (vertical lines = events)');
addEventMarkers(tvec, evtMark);

nexttile; hold on; grid on;
plot(tvec, rad2deg(abs(epsih)), 'LineWidth', 1.6);
ylabel('|e_\psi| [deg]'); xlabel('Time [s]');
addEventMarkers(tvec, evtMark);

fprintf('\n===== FINAL Loop disturbance test summary =====\n');
fprintf('Events: %d\n', sum(evtMark));
fprintf('Max |e_ct|: %.3f m, Mean |e_ct|: %.3f m\n', max(abs(ect)), mean(abs(ect)));
fprintf('Max |e_psi|: %.2f deg, Mean |e_psi|: %.2f deg\n', max(rad2deg(abs(epsih))), mean(rad2deg(abs(epsih))));
