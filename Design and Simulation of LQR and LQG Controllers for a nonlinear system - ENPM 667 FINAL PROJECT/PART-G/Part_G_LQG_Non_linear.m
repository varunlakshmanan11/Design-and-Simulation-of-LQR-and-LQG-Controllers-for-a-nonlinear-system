% Part_G_LQG_Non-Linear

% Define System Parameters
M = 1000; % Mass of the base
m1 = 100; % Mass of pendulum 1
m2 = 100; % Mass of pendulum 2
l1 = 20;  % Length of pendulum 1
l2 = 10;  % Length of pendulum 2
g = 9.81; % Acceleration due to gravity

% State-space Matrices for the Linearized Model
A = [0 1 0 0 0 0; 
     0 0 -(g*m1)/M 0 -(g*m2)/M 0;
     0 0 0 1 0 0;
     0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
     0 0 0 0 0 1;
     0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0]; % A matrix

B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

C = [1 0 0 0 0 0]; % Measurement matrix, measuring only position 'x'

% LQR Controller Design
Q = [20 0 0 0 0 0;
     0 200 0 0 0 0;
     0 0 1000 0 0 0;
     0 0 0 1000 0 0;
     0 0 0 0 200 0;
     0 0 0 0 0 20];
R = 0.005;
[K, ~, ~] = lqr(A, B, Q, R);

% Designing State Estimator using Kalman Filter
% Defining process noise covariance (W) and measurement noise covariance (V)
W = 0.1 * eye(6); 
V = 0.1;         
[L, ~, ~] = lqe(A, eye(6), C, W, V);

% Initial Conditions for the Nonlinear System
x_init = [0; 0; 20; 0; 40; 0]; % Initial state

% Time Span for Simulation
tspan = [0 20];
step_time = 1; % Time for step input

% Nonlinear System Simulation using LQG Controller
[t, y] = ode45(@(t, x) nonlinear_system_with_lqg(t, x, K, L, A, B, C, M, m1, m2, l1, l2, g, step_time, V), tspan, x_init);

% Extracting control input and noise from the output
control_input = y(:, 2);
noise_array = y(:, 6);

% Plot Results

% Plot Results
figure;
% Plotting the position 'x' over time
subplot(6,1,1);
plot(t, y(:, 1));
xlabel('Time (s)');
ylabel('Position x');
title('Position of x vs. Time');
grid on;

% Plotting other state variables
subplot(6,1,2);
plot(t, y(:, 2));
xlabel('Time (s)');
ylabel('Velocity of x');
title('Velocity of x vs. Time');
grid on;

subplot(6,1,3);
plot(t, y(:, 3));
xlabel('Time (s)');
ylabel('Angle of mass 1');
title('Angle of mass 1 vs. Time');
grid on;

subplot(6,1,4);
plot(t, y(:, 4));
xlabel('Time (s)');
ylabel('Angular Velocity of mass 1');
title('Angular Velocity of mass 1 vs. Time');
grid on;

subplot(6,1,5);
plot(t, y(:, 5));
xlabel('Time (s)');
ylabel('Angle of mass 2');
title('Angle of mass 2 vs. Time');
grid on;

subplot(6,1,6);
plot(t, y(:, 6));
xlabel('Time (s)');
ylabel('Angular Velocity of mass 2');
title('Angular Velocity of mass 2 vs. Time');
grid on;

% Function defining nonlinear system dynamics with LQG controller
function [diff, u, noise] = nonlinear_system_with_lqg(t, x, K, L, A, B, C, M, m1, m2, l1, l2, g, step_time, V)
    y = C * x;
    x_hat = A * x + B * (-K * x) + L * (y - C * x);

    % Unit step input
    step_input = (t >= step_time);

    % Control input
    u = -K * x_hat + step_input;

    % Simulated noise 
    noise = randn * sqrt(V); % Using the measurement noise covariance 'V'

    % System dynamics eqution of non linear system
    diff = zeros(6, 1);
    diff(1) = x(2);
    diff(2) = (u - (g/2)*(m1*sin(2*x(3)) + m2*sin(2*x(5))) - (m1*l1*x(4)^2*sin(x(3))) - (m2*l2*x(6)^2*sin(x(5)))) / (M + m1*sin(x(3))^2 + m2*sin(x(5))^2);
    diff(3) = x(4);
    diff(4) = (diff(2)*cos(x(3)) - g*sin(x(3))) / l1;
    diff(5) = x(6);
    diff(6) = (diff(2)*cos(x(5)) - g*sin(x(5))) / l2;
end
