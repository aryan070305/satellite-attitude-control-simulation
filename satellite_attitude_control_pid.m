clc;
clear;
close all;

% System parameters
I = 5;              % Moment of inertia
s = tf('s');        % Laplace variable
G = 1/(I*s^2);      % Transfer function

disp('Open Loop Transfer Function:')
G

% Open loop step response
figure;
step(G);
title('Open Loop Step Response (No Controller)');
xlabel('Time ');
ylabel('Angular Position (rad)');
grid on;

% PID controller gains
Kp = 12;      
Ki = 1;       
Kd = 18;       

N =  80;           % Derivative filter parameter
Tf = 1/N;          % Filter time constant

% Create PID controller
PID_Controller = pid(Kp, Ki, Kd,Tf);

% Closed-loop system with unity feedback
Closed_Loop_System = feedback(PID_Controller * G, 1);

% Closed-loop step response
figure;
step(Closed_Loop_System);
title('Closed Loop Step Response with PID Controller');
xlabel('Time ');
ylabel('Angular Position (rad)');
grid on;

% Get performance metrics
Performance_Metrics = stepinfo(Closed_Loop_System);

% Display results
disp('Performance Metrics:')
Performance_Metrics

% Time vector for simulation
t = 0:0.01:20;

% Sinusoidal disturbance torque
disturbance = 0.2 * sin(0.5 * t);

% Disturbance to output transfer function
Disturbance_Response = feedback(G, PID_Controller);

% Simulate disturbance response
[y, t_out] = lsim(Disturbance_Response, disturbance, t);

% Plot disturbance response
figure;
plot(t_out, y, 'LineWidth', 1.5);
title('Satellite Response to External Disturbance');
xlabel('Time (seconds)');
ylabel('Angular Position (rad)');
grid on;

%% Visual Animation

% Closed-loop step response data
[theta, t] = step(Closed_Loop_System);

figure;

for k = 1:length(t)

    % Current rotation angle
    current_angle = theta(k);

    % Satellite arm length
    L = 1;

    % Rotated coordinates
    x = L * cos(current_angle);
    y = L * sin(current_angle);

   clf;

% Draw satellite arm
plot([0 x], [0 y], 'LineWidth', 3);
hold on;

% Reference axis
plot([-1.2 1.2], [0 0], '--k');

% Origin point
plot(0,0,'ko','MarkerSize',6,'MarkerFaceColor','k');

axis equal;
axis([-1.2 1.2 -1.2 1.2]);

xlabel('X Axis');
ylabel('Y Axis');
title(['Satellite Attitude (Theta = ' num2str(current_angle,3) ' rad)']);

grid on;

    if k > 1
    pause(t(k) - t(k-1));
end

end
