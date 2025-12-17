clear; clc; close all;
addpath(genpath(fullfile(fileparts(mfilename('fullpath')), '..', 'src')));
%% RRT planning (animated) + path tracking (animated)
clear; clc; close all;

%% -------------------- Workspace --------------------
xmin = 0;  xmax = 20;
ymin = 0;  ymax = 15;

start = [2.0, 2.0];
goal  = [18.0, 13.0];

%% -------------------- Obstacle field --------------------
numObs = 32;
rMin   = 0.18;
rMax   = 0.55;
margin = 0.35;          % safety inflation

rng(11);

%% -------------------- RRT parameters --------------------
maxIters      = 14000;
stepSize      = 0.35;
goalRadius    = 0.7;
goalBias      = 0.10;
collisionStep = 0.10;

% Animation (planning)
planPause  = 0.01;
planStride = 1;

% Animation (tracking)
movePause = 0.0;

%% -------------------- Generate obstacles --------------------
obs = generateObstacles(numObs, [xmin xmax ymin ymax], [start; goal], rMin, rMax);

%% -------------------- Plot map --------------------
figure('Name','RRT Planning (Animated) + Tracking');
hold on; axis equal; grid on;
xlim([xmin xmax]); ylim([ymin ymax]);
xlabel('X [m]'); ylabel('Y [m]');
title('RRT planning');

% Obstacles (single legend entry)
hObsLeg = []; hMarLeg = [];
for i = 1:size(obs,1)
    cx = obs(i,1); cy = obs(i,2); r = obs(i,3);
    hObs = drawCircleFilled(cx, cy, r);
    hMar = drawCircleOutline(cx, cy, r+margin, 'r');

    if i==1
        set(hObs,'DisplayName','Obstacle');
        set(hMar,'DisplayName','Safety margin');
        hObsLeg = hObs; hMarLeg = hMar;
    else
        set(hObs,'HandleVisibility','off');
        set(hMar,'HandleVisibility','off');
    end
end

hStart = plot(start(1), start(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName','Start');
hGoal  = plot(goal(1),  goal(2),  'mo', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName','Goal');

% RRT tree + animation markers
hTree    = animatedline('LineStyle','-','LineWidth',0.7,'DisplayName','RRT tree');
hNewNode = plot(nan,nan,'co','MarkerSize',6,'LineWidth',1.5,'HandleVisibility','off');
hNewEdge = plot(nan,nan,'c-','LineWidth',1.2,'HandleVisibility','off');

% Planned and actual paths (legend fixed)
hPlanned = plot(nan,nan,'g--','LineWidth',2,'DisplayName','Planned path (RRT)');
hActual  = plot(nan,nan,'b-','LineWidth',2,'DisplayName','Actual path');
hRobot   = plot(nan,nan,'bd','MarkerSize',8,'LineWidth',2,'HandleVisibility','off');

lgd = legend([hObsLeg,hMarLeg,hStart,hGoal,hTree,hPlanned,hActual], ...
    {'Obstacle','Safety margin','Start','Goal','RRT tree','Planned path (RRT)','Actual path'}, ...
    'Location','bestoutside');
lgd.AutoUpdate = 'off';

%% -------------------- RRT planning --------------------
nodes  = start;    % Nx2
parent = 0;        % parent index per node
found  = false;
goalIdx = -1;

for it = 1:maxIters
    % Sample (goal-biased)
    if rand < goalBias
        qRand = goal;
    else
        qRand = [xmin + (xmax-xmin)*rand(), ymin + (ymax-ymin)*rand()];
    end

    % Nearest node
    [idxNear, qNear] = nearestNode(nodes, qRand);

    % Steer
    qNew = steerToward(qNear, qRand, stepSize);

    % Boundary check
    if qNew(1)<xmin || qNew(1)>xmax || qNew(2)<ymin || qNew(2)>ymax
        continue;
    end

    % Collision check with inflated obstacles
    if ~isSegmentCollisionFree(qNear, qNew, obs, margin, collisionStep)
        continue;
    end

    % Add node
    nodes(end+1,:) = qNew; %#ok<AGROW>
    parent(end+1,1) = idxNear; %#ok<AGROW>

    % Animate tree growth
    if mod(size(nodes,1), planStride) == 0
        addpoints(hTree, [qNear(1) qNew(1) nan], [qNear(2) qNew(2) nan]);
        set(hNewNode, 'XData', qNew(1), 'YData', qNew(2));
        set(hNewEdge, 'XData', [qNear(1) qNew(1)], 'YData', [qNear(2) qNew(2)]);
        drawnow limitrate;
        pause(planPause);
    end

    % Goal check
    if norm(qNew - goal) <= goalRadius
        if isSegmentCollisionFree(qNew, goal, obs, margin, collisionStep)
            nodes(end+1,:) = goal; %#ok<AGROW>
            parent(end+1,1) = size(nodes,1)-1; %#ok<AGROW>
            addpoints(hTree, [qNew(1) goal(1) nan], [qNew(2) goal(2) nan]);
            drawnow;
            goalIdx = size(nodes,1);
            found = true;
        else
            goalIdx = size(nodes,1);
            found = true;
        end
        break;
    end
end

if ~found
    error('RRT failed. Increase maxIters/goalBias or reduce obstacles/margin.');
end

%% -------------------- Planned path --------------------
pathXY = backtrackPath(nodes, parent, goalIdx);
pathXY(end,:) = goal(:).';

set(hPlanned,'XData',pathXY(:,1),'YData',pathXY(:,2));
title('Planning done. Tracking...');
drawnow;

%% -------------------- Tracking (4WS-4WD + Pure Pursuit) --------------------
% States (world): [x,y,psi], [xd,yd,psid]
x   = start(1);
y   = start(2);
psi = atan2(pathXY(2,2)-pathXY(1,2), pathXY(2,1)-pathXY(1,1));
xd = 0; yd = 0; psid = 0;

dt   = 0.02;
Tmax = 300;

% Vehicle
m  = 25;
Iz = 2.0;
a  = 0.35;
b  = 0.25;

Rw = 0.06;
Jw = 0.02;
bw = 0.03;

Rpos = [[+a;+b],[+a;-b],[-a;+b],[-a;-b]];   % FL FR RL RR

mu = 0.9;
g  = 9.81;
Nw = m*g/4;

% Pure Pursuit
Ld   = 0.40;
vMax = 1.0;
wMax = 2.5;

% Wheel PI
Kp_w   = 0.6;
Ki_w   = 0.4;
tauMax = 2.5;
Imax   = 15;

% Slip-speed tire + damping
Cv = 180;
cV = 0.7;
cR = 0.5;

snapRadius = 0.07;
lastIdx = 1;

omega_w = zeros(4,1);
int_w   = zeros(4,1);

% Plot init
actual = [x y];
set(hActual,'XData',actual(:,1),'YData',actual(:,2));
set(hRobot,'XData',x,'YData',y);

t = 0;
title('Tracking (Pure Pursuit + 4WS/4WD dynamics)...');
drawnow;

while t < Tmax
    t = t + dt;

    % Stop near goal
    distEnd = hypot(x - pathXY(end,1), y - pathXY(end,2));
    if distEnd < snapRadius && hypot(xd,yd) < 0.05 && abs(psid) < 0.2
        x = pathXY(end,1); y = pathXY(end,2);
        actual(end+1,:) = [x y]; %#ok<AGROW>
        break;
    end

    % Lookahead reference
    [xref, yref, ~, lastIdx] = getRefOnPathForward([x y], pathXY, Ld, lastIdx);

    N = size(pathXY,1);
    if lastIdx >= N-3
        xref = pathXY(end,1);
        yref = pathXY(end,2);
    end

    % Pure Pursuit -> (v_des, w_des)
    dxg = xref - x; dyg = yref - y;
    alpha = wrapToPi_local(atan2(dyg, dxg) - psi);
    ePos  = hypot(dxg, dyg);

    v_des = min(vMax, 0.9*ePos);
    kappa_pp = 2*sin(alpha) / max(Ld, 1e-3);
    w_des = max(min(v_des*kappa_pp, wMax), -wMax);
    v_des = min(v_des, vMax/(1 + 0.9*abs(w_des)));

    vx_des = v_des; vy_des = 0; om_des = w_des;

    % Current body velocity
    Rwb = [ cos(psi)  sin(psi);
           -sin(psi)  cos(psi) ];
    v_body = Rwb * [xd; yd];
    vx = v_body(1); vy = v_body(2); om = psid;

    % 4WS allocation
    delta = zeros(4,1);
    vcmd  = zeros(4,1);
    vlong = zeros(4,1);

    for i = 1:4
        xi = Rpos(1,i); yi = Rpos(2,i);

        vix_des = vx_des - om_des*yi;
        viy_des = vy_des + om_des*xi;

        delta(i) = atan2(viy_des, vix_des);
        vcmd(i)  = hypot(vix_des, viy_des);

        vix = vx - om*yi;
        viy = vy + om*xi;

        h = [cos(delta(i)); sin(delta(i))];
        vlong(i) = h.' * [vix; viy];
    end

    omega_des = vcmd / Rw;

    % Wheel PI + tire force
    F = zeros(4,1);
    for i = 1:4
        ew = omega_des(i) - omega_w(i);
        int_w(i) = max(min(int_w(i) + ew*dt, Imax), -Imax);

        tau = Kp_w*ew + Ki_w*int_w(i);
        tau = max(min(tau, tauMax), -tauMax);

        s = Rw*omega_w(i) - vlong(i);
        F(i) = Cv * s;
        F(i) = max(min(F(i), mu*Nw), -mu*Nw);

        omega_wdot = (tau - Rw*F(i) - bw*omega_w(i)) / Jw;
        omega_w(i) = omega_w(i) + omega_wdot*dt;
    end

    % Rigid-body dynamics (world frame)
    B = zeros(3,4);
    for i = 1:4
        B(1,i) = cos(psi + delta(i));
        B(2,i) = sin(psi + delta(i));
        B(3,i) = Rpos(1,i)*sin(delta(i)) - Rpos(2,i)*cos(delta(i));
    end

    qdd = [ (B(1,:)*F)/m;
            (B(2,:)*F)/m;
            (B(3,:)*F)/Iz ];

    qdd(1) = qdd(1) - cV*xd;
    qdd(2) = qdd(2) - cV*yd;
    qdd(3) = qdd(3) - cR*psid;

    xd   = xd   + qdd(1)*dt;
    yd   = yd   + qdd(2)*dt;
    psid = psid + qdd(3)*dt;

    x    = x    + xd*dt;
    y    = y    + yd*dt;
    psi  = wrapToPi_local(psi + psid*dt);

    % Plot update
    actual(end+1,:) = [x y]; %#ok<AGROW>
    set(hActual, 'XData', actual(:,1), 'YData', actual(:,2));
    set(hRobot,  'XData', x, 'YData', y);
    drawnow limitrate;
    pause(movePause);
end

title(sprintf('Done: Planned vs Actual, t=%.1fs', t));
hold off;

%% -------- local wrapToPi (no toolbox) --------
function ang = wrapToPi_local(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
