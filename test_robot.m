%% parameters
%%% robot
param.m = 10.1;
param.J = 0.13;
param.L = 0.12;
%%% control mode
param.mode = ControlMode.Default;
%%% goal distance tolerance
param.h_tol = 0.1;
%%% control law
param.Kg = 0.5 * eye(2);
param.Dg = 1 * eye(2);
param.Kf = 5 * eye(2);
param.Df = 1 * eye(2);
%%% number of robots
N = 3;
%%% initial conditions
x0 = [0  ; -2; pi/2; 0; 0;
      0.5; -2; pi/2; 0; 0;
     -0.5; -2; pi/2; 0; 0];
%%% trajectory.
h_ref = [[0; -3; 1; -4  ; -1; -4],...
         [0;  4; 2;  2  ; -2;  2],...
         [5;  6; 5;  4.5; 5; 3]] / 3;         
%%% simulation
duration = 15;
Ts = 0.1;

%% simulate
t = 0:Ts:duration;
ode_fcn = @(t,x) closed_loop_ode(t, x, h_ref, param);
[t,x] = ode45(ode_fcn, t, x0);

%% plot
rx = x(:,1:5:end);
ry = x(:,2:5:end);
theta = x(:,3:5:end);

Ts = mean(diff(t));

colors = num2cell(colororder, 2);

figure(1)
clf
hold on
for k = 1:N
    plot(rx(:,k), ry(:,k), 'Color', colors{k})
    scatter([x0(5*k-4), h_ref(2*k-1,:)], [x0(5*k-3), h_ref(2*k,:)], 150, 'x', ...
        'LineWidth', 1.5, 'MarkerEdgeColor', colors{k})
end
r = cell(N, 1);
for k = 1:N
    r{k} = robot_plot(0,0,0,'HandLength',param.L,'BodyColor',colors{k});
end
xlim([min(rx,[],'all')-0.2,max(rx,[],'all')+0.2])
ylim([min(ry,[],'all')-0.2,max(ry,[],'all')+0.2])
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