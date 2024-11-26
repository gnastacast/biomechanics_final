
% model parameters
m = 80; % [kg]
g = 9.81; % [m/s^2]
p0 = 0.4; % [m]
l0 = 1 - p0; % [m]
k = 20000; % [N/m]

a0 = 68 * pi/180;  % [rad] : angle of attack during flight

% initial conditions
x0 = 0;
dx0 = 5;
y0 = 1;
dy0 = 0; % start at apex in flight

model_version = 1;
noise_seed = 0;
noise_gain = 0;

load('polys.mat')
out = sim('model5_2f.slx');

%%
figure();
dt = gradient(out.xy.Time);
dy_dt = gradient(out.xy.Data(:,2)) ./ dt;
ddy_ddt = diff(dy_dt) ./ dt(2:end);
ddy_ddt = [0; ddy_ddt];

plot(out.xy.Time, dy_dt)

% ylim([-40,40])

%%
figure;
xyFP_vec = out.xyFP.Data;
% inFlight_vec = out.yout{2}.Values.data;
xy_vec = out.xy.Data;
noise = rand(size(xy_vec)) * .1 .* [1 1];
xy_vec = xy_vec + noise;
hold on
% plot(xy_vec(:,1), xy_vec(:,2), "-or");
% plot(xyFP_vec(:,1), xyFP_vec(:,2), "-o");
% plot(xy_vec(1,:) + noise(1,:), xy_vec(2,:) + noise(2,:));

plot(out.xy.Time, 2 + (abs(xyFP_vec(:,2)) <= 0.0001))
plot(out.in_stance.Time, out.in_stance.Data)

axis equal

% %%
% 
% sim('./ctrlKalmanNavigationExample.slx');
% 
% figure;
% % Plot results and connect data points with a solid line.
% plot(x(:,1),x(:,2),'bx',...
%     y(:,1),y(:,2),'gd',...
%     xhat(:,1),xhat(:,2),'ro',...
%     'LineStyle','-');
% title('Position');
% xlabel('East [m]');
% ylabel('North [m]');
% legend('Actual','Measured','Kalman filter estimate','Location','Best');
% axis tight;

%%
smoothedXYvec = smoothdata(xy_vec,'movmean');
vxy = smoothdata(diff(smoothedXYvec), 'movmean');
apexPoints = (vxy(1:end-1,2) > 0) & (vxy(2:end,2) < 0); % Find maxima
apexIndices = find(apexPoints);

dx = []; dy = []; h = [];
takeoffIndices = [];

for i = 1:length(xyFP_vec(:,2))-1
    if xyFP_vec(i,2) <= 0 && xyFP_vec(i+1,2) > 0
        fprintf('Current index i: %d\n', i)
        takeoffIndices(end+1) = i;
        dx(end+1) = vxy(i+1,1);
        dy(end+1) = vxy(i+1,2); % velocity at takeoff
    end
end

apexPos = zeros(length(apexIndices), 2);
apexTimes = [];
% apex(1) = x + dx * t; % x component
% apex(2) = y + dy * t + 0.5 * g * t ^ 2; % y component
for i = 1:length(apexIndices)
    apexPoint = apexIndices(i);
    takeoffPoint = takeoffIndices(i);
    t = out.xy.Time(apexPoint) - out.xy.Time(takeoffPoint); % time to reach apex from takeoff
    apexTimes(end+1) = out.xy.Time(takeoffPoint) + t; % Store times for plot / visualization
    apexPos(i,1) = xy_vec(takeoffPoint,1) + dx(i) * t; % X pos
    apexPos(i,2) = xy_vec(takeoffPoint,2) + dy(i) * t - 0.5 * g * t ^ 2; % Y pos
end

% 1/2mv^2 = mgh -> h = v^2/2g
for i = 1:length(dy)
    h(end+1) = dy(i)^2 / 2 * g;
end

figure;
grid on;
plot(out.xy.Time(2:end), vxy(:,2));
title('Smoothed Velocity vs Time');
xlabel('Time (s)');
ylabel('v_y');

figure;
grid on;
subplot(2, 1, 1);
hold on;
plot(out.xy.Time, smoothedXYvec(:,1));
plot(apexTimes, apexPos(:,1), 'o', 'MarkerSize', 5, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
title('X Position vs Time');
xlabel('Time (s)');
ylabel('X Position (m)');
legend('X Position', 'Apex X Points');

subplot(2, 1, 2);
hold on;
plot(out.xy.Time, smoothedXYvec(:,2));
plot(apexTimes, apexPos(:,2), 'o', 'MarkerSize', 5, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b');
title('Y Position vs Time');
xlabel('Time (s)');
ylabel('Y Position (m)');
legend('Y Position', 'Apex Y Points')