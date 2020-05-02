%% parameters
%%% robot
param.m = 1;
param.J = 1;
param.L = 0.5;
%%% control mode
param.mode = ControlMode.Default;
%%% control law
param.Kg = 50 * eye(2);
param.Dg = 50 * eye(2);
param.Kf = 5 * eye(2);
param.Df = 5 * eye(2);
%%% number of robots
N = 3;
%%% position reference
h_ref = [4;4;6;8;8;4];
%%% initial condition
x01 = [0;0;pi;0;0];
x02 = [1;1;-pi/2;0;0];
x03 = [2;0;0;0;0];
x0 = [x01;x02;x03];

%% simulate
t = linspace(0, 10, 101);
ode_fcn = @(~,x) closed_loop_ode(x, h_ref, param);
[t,x] = ode45(ode_fcn, t, x0);

%% plot
rx = x(:,1:5:end);
ry = x(:,2:5:end);
theta = x(:,3:5:end);

Ts = mean(diff(t));

figure(1)
clf
hold on
for k = 1:N
    plot(rx(:,k), ry(:,k))
end
r = cell(N, 1);
for k = 1:N
    r{k} = robot_plot(0,0,0,'HandLength',param.L);
end
xlim([min(rx,[],'all')-1,max(rx,[],'all')+1])
ylim([min(ry,[],'all')-1,max(ry,[],'all')+1])
for k = 1:numel(t)
    tic
    for j = 1:N
        r{j}.x = rx(k,j);
        r{j}.y = ry(k,j);
        r{j}.theta = theta(k,j);
    end
    title(sprintf('Time %.02f', t(k)))
    pause(Ts - toc)
end