% Part_F_Luenberger_observer_non-linear_system


% Define System Parameters
m1=100; % in kg
m2=100; % in kg
M=1000; % in kg
l1=20; % in meters
l2=10; % in meters
g=9.81; % m/s

% State-Space Matrices
A = [0 1 0 0 0 0; 
     0 0 -(m1*g)/M 0 -(m2*g)/M 0;
     0 0 0 1 0 0;
     0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
     0 0 0 0 0 1;
     0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];

B = [0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

% Output Matrices for Observable Cases
C1 = [1 0 0 0 0 0];% C1 matrix to monitor x(t)
C3 = [1 0 0 0 0 0; 0 0 0 0 1 0];% C3 matrix to monitor x(t), theta2(t) 
C4 = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];% C4 matrix to monitor x(t), theta1(t), theta2(t) 
% Lqr  
Q = diag([10 200 2000 1000 200 10]);%Used diag function to mention the diagonal components in the Q matrix.
R = 0.05;
K = lqr(A,B,Q,R); % using in built function for lqr controlller

% Design of Luenberger Observer
poles = -1:-1:-6;
L1 = place(A', C1', poles)'; % Luenberger matrix L1 by pole placement
L3 = place(A', C3', poles)'; %Luenberger matrix L3
L4 = place(A', C4', poles)'; %Luenberger matrix L4

% Define Systems for Simulation
systems = {
    struct('A', A - B * K, 'B', B, 'C', C1, 'L', L1);
    struct('A', A - B * K, 'B', B, 'C', C3, 'L', L3);
    struct('A', A - B * K, 'B', B, 'C', C4, 'L', L4)
};

% Initial Conditions and 6 estimate values
x_init = [0, 0, 20, 0, 40, 0, 0, 0, 0, 0, 0, 0];

% loop for finding augumented matrix and plot simulations
for i = 1:length(systems)
    sys = systems{i};
    A_aug = [sys.A sys.B * K; zeros(size(sys.A)) sys.A - sys.L * sys.C];
    B_aug = [sys.B; zeros(size(sys.B))];
    C_aug = [sys.C zeros(size(sys.C))];
    D = 0;

    ss_sys = ss(A_aug, B_aug, C_aug, D);
    
    figure;
    initial(ss_sys, x_init);
    title(sprintf('Initial State Response for System %d', i));
    grid on;
    
    figure;
    step(ss_sys);
    title(sprintf('Step State Response for System %d', i));
    grid on;
end
