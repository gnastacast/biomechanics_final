
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