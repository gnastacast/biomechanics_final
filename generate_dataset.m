
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
