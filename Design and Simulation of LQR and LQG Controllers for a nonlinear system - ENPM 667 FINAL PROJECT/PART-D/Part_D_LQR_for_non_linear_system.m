% Part_D_LQR for non-linear system

%Defining the output intial variables
y_intial = [0; 0; 20; 0; 40; 0];
tspan = 0:0.01:4000;% tspan is the time span for th system
[t_new,y_new] = ode45(@nonlinear,tspan,y_intial); %using ode45 function for non-linear systems
plot(t_new,y_new)
grid on

% Defining non-linear function to call during the ode45 function
function dydt = nonlinear(t,y)

m1=100; % in kg
m2=100; % in kg
M=1000; % in kg
l1=20; % in meters
l2=10; % in meters
g=9.81; % m/s

A=[0 1 0 0 0 0; 
    0 0 -(m1*g)/M 0 -(m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -(m1*g)/(M*l2) 0 -(g*(M+m2))/(M*l2) 0];
B=[0; 1/M; 0; 1/(M*l1); 0; 1/(M*l2)];

Q=[20 0 0 0 0 0;
   0 200 0 0 0 0;
   0 0 1000 0 0 0;
   0 0 0 1000 0 0;
   0 0 0 0 200 0;
   0 0 0 0 0 20];
R=0.0005;

[K, P, Poles] = lqr(A,B,Q,R); % using in built function for lqr controlller

disp("P:")
disp(P);

disp("K:")
disp(K);

disp("Poles:")
disp(Poles);

% Using the optimal gain matrix K, to get the force for the equation to
% find state feedback
F=-K*y;

dydt=zeros(6,1);
dydt(1) = y(2); %XD
dydt(2)=(F-(g/2)*(m1*sind(2*y(3))+m2*sind(2*y(5)))-(m1*l1*(y(4)^2)*sind(y(3)))-(m2*l2*(y(6)^2)*sind(y(5))))/(M+m1*((sind(y(3)))^2)+m2*((sind(y(5)))^2));%xDD
dydt(3)= y(4); %theta 1D
dydt(4)= (dydt(2)*cosd(y(3))-g*(sind(y(3))))/l1'; %theta 1 Ddot;
dydt(5)= y(6); %theta 2D
dydt(6)= (dydt(2)*cosd(y(5))-g*(sind(y(5))))/l2; %theta 2Ddot;
end
