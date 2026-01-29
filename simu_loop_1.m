clear all
close all
clc

% ==========================================
% 1. PARAMETERS
% ==========================================
L1 = 0.210; % Ground (Pink)
L2_Loop1 = 0.180; % Green
L3_Loop1 = 0.180; % Yellow
L4_Shared = 0.118; % Grey

L2_Loop2 = 0.118; % Cyan
L3_Loop2 = 0.210; % Red

L2_Loop3 = 0.118; % Cyan (Extended)
L3_Loop3 = 0.210; % Blue
L4_Loop3 = 0.118; % Brown

d = L1;
offset_deg = 0.81;
offset = deg2rad(offset_deg);

% ==========================================
% 2. SIMULATION SETUP
% ==========================================
t_final = 1;            % Duration (seconds)
dt = 0.01;              % Time step
time = 0:dt:t_final;    % Time vector
N = length(time);

% Initial Input Conditions (Yellow Link)
q3_init_deg = 19.94;
q3_init = deg2rad(q3_init_deg);
w_init = 0.03;          % rad/s
alpha_input = 0.003;    % rad/s^2

% Storage Arrays
History_Angles = zeros(N, 7);      % [Green, Yellow, Grey, Cyan, Red, Blue, Brown]
History_Velocities = zeros(N, 7);
History_Accelerations = zeros(N, 7);
History_P = zeros(N, 2);           % [Px, Py]

% ==========================================
% 3. TIME LOOP ANALYSIS
% ==========================================
for i = 1:N
    t = time(i);
    
    % --- A. UPDATE INPUTS (Kinematics Equations) ---
    % theta = theta0 + w0*t + 0.5*a*t^2
    q3_global = q3_init + w_init*t + 0.5*alpha_input*t^2;
    q3 = q3_global - offset; % Local angle
    
    % w = w0 + a*t
    w_Yellow = w_init + alpha_input*t;
    
    % alpha = constant
    alpha_Yellow = alpha_input;
    
    % --- B. POSITION ANALYSIS ---
    
    % LOOP 1: Green-Yellow-Grey
    a = L2_Loop1; b = L3_Loop1; c = L4_Shared;
    K1=d/b; K2=d/c; K3=(b^2-a^2+c^2+d^2)/(2*b*c); K4=d/a; K5=(c^2-d^2-b^2-a^2)/(2*b*a);
    A=cos(q3)-K1-K2*cos(q3)+K3; B=-2*sin(q3); C=K1-(K2+1)*cos(q3)+K3;
    D=cos(q3)-K1+K4*cos(q3)+K5; E=-2*sin(q3); F=K1+(K4-1)*cos(q3)+K5;
    
    % Using Case 1 (Open)
    q4_Grey = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));
    q2_Green = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));
    
    % LOOP 2: Cyan-Red-Grey
    a = L2_Loop2; b = L3_Loop2; c = L4_Shared;
    K1_L2=d/c; K2_L2=d/a; K3_L2=(c^2-b^2+a^2+d^2)/(2*c*a); K4_L2=d/b; K5_L2=(a^2-d^2-c^2-b^2)/(2*c*b);
    
    q_in_1 = q4_Grey;
    A2=cos(q_in_1)-K1_L2-K2_L2*cos(q_in_1)+K3_L2; B2=-2*sin(q_in_1); C2=K1_L2-(K2_L2+1)*cos(q_in_1)+K3_L2;
    D2=cos(q_in_1)-K1_L2+K4_L2*cos(q_in_1)+K5_L2; E2=-2*sin(q_in_1); F2=K1_L2+(K4_L2-1)*cos(q_in_1)+K5_L2;
    
    q2_Cyan = 2*atan((-B2 + sqrt(B2^2 - 4*A2*C2))/(2*A2)); 
    q3_Red = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));
    
    % LOOP 3: Cyan-Blue-Brown
    a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;
    K1_L3=d/a; K2_L3=d/c; K3_L3=(a^2-b^2+c^2+d^2)/(2*a*c); K4_L3=d/b; K5_L3=(c^2-d^2-a^2-b^2)/(2*a*b);
    
    q_in_3 = q2_Cyan + pi;
    A3=cos(q_in_3)-K1_L3-K2_L3*cos(q_in_3)+K3_L3; B3=-2*sin(q_in_3); C3=K1_L3-(K2_L3+1)*cos(q_in_3)+K3_L3;
    D3=cos(q_in_3)-K1_L3+K4_L3*cos(q_in_3)+K5_L3; E3=-2*sin(q_in_3); F3=K1_L3+(K4_L3-1)*cos(q_in_3)+K5_L3;
    
    q4_Brown = 2*atan((-B3 + sqrt(B3^2 - 4*A3*C3))/(2*A3));
    q3_Blue = 2*atan((-E3 + sqrt(E3^2 - 4*D3*F3))/(2*D3));
    
    % --- C. VELOCITY ANALYSIS ---
    % Loop 1
    a=L2_Loop1; b=L3_Loop1; c=L4_Shared;
    w_Grey = (b * w_Yellow * sin(q3 - q2_Green)) / (c * sin(q4_Grey - q2_Green));
    w_Green = (b * w_Yellow * sin(q4_Grey - q3)) / (a * sin(q4_Grey - q2_Green));
    
    % Loop 2
    a=L2_Loop2; b=L3_Loop2; c=L4_Shared;
    w_Cyan = (c * w_Grey * sin(q4_Grey - q3_Red)) / (a * sin(q2_Cyan - q3_Red));
    w_Red = (c * w_Grey * sin(q4_Grey - q2_Cyan)) / (b * sin(q3_Red - q2_Cyan));
    
    % Loop 3
    a=L2_Loop3; b=L3_Loop3; c=L4_Loop3;
    w_Brown = (a * w_Cyan * sin(q_in_3 - q3_Blue)) / (c * sin(q4_Brown - q3_Blue));
    w_Blue = (a * w_Cyan * sin(q4_Brown - q_in_3)) / (b * sin(q4_Brown - q3_Blue));
    
    % --- D. ACCELERATION ANALYSIS ---
    % Loop 1
    a = L2_Loop1; b = L3_Loop1; c = L4_Shared;
    A_coeff = -a*sin(q2_Green); B_coeff = c*sin(q4_Grey);
    D_coeff = a*cos(q2_Green);  E_coeff = -c*cos(q4_Grey);
    C_val = a*w_Green^2*cos(q2_Green) + b*w_Yellow^2*cos(q3) + b*alpha_Yellow*sin(q3) - c*w_Grey^2*cos(q4_Grey);
    F_val = a*w_Green^2*sin(q2_Green) + b*w_Yellow^2*sin(q3) - b*alpha_Yellow*cos(q3) - c*w_Grey^2*sin(q4_Grey);
    
    alpha_Green = (C_val*E_coeff - B_coeff*F_val) / (A_coeff*E_coeff - B_coeff*D_coeff);
    alpha_Grey  = (A_coeff*F_val - C_val*D_coeff) / (A_coeff*E_coeff - B_coeff*D_coeff);
    
    % Loop 2
    a = L2_Loop2; b = L3_Loop2; c = L4_Shared;
    A_coeff = -a*sin(q2_Cyan); B_coeff = -b*sin(q3_Red);
    D_coeff = a*cos(q2_Cyan);  E_coeff = b*cos(q3_Red);
    C_val = a*w_Cyan^2*cos(q2_Cyan) + b*w_Red^2*cos(q3_Red) - c*w_Grey^2*cos(q4_Grey) - c*alpha_Grey*sin(q4_Grey);
    F_val = a*w_Cyan^2*sin(q2_Cyan) + b*w_Red^2*sin(q3_Red) - c*w_Grey^2*sin(q4_Grey) + c*alpha_Grey*cos(q4_Grey);
    
    alpha_Cyan = (C_val*E_coeff - B_coeff*F_val) / (A_coeff*E_coeff - B_coeff*D_coeff);
    alpha_Red  = (A_coeff*F_val - C_val*D_coeff) / (A_coeff*E_coeff - B_coeff*D_coeff);
    
    % Loop 3
    a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;
    A_coeff = -b*sin(q3_Blue); B_coeff = c*sin(q4_Brown);
    D_coeff = b*cos(q3_Blue);  E_coeff = -c*cos(q4_Brown);
    C_val = a*w_Cyan^2*cos(q_in_3) + a*alpha_Cyan*sin(q_in_3) + b*w_Blue^2*cos(q3_Blue) - c*w_Brown^2*cos(q4_Brown);
    F_val = a*w_Cyan^2*sin(q_in_3) - a*alpha_Cyan*cos(q_in_3) + b*w_Blue^2*sin(q3_Blue) - c*w_Brown^2*sin(q4_Brown);
    
    alpha_Blue  = (C_val*E_coeff - B_coeff*F_val) / (A_coeff*E_coeff - B_coeff*D_coeff);
    alpha_Brown = (A_coeff*F_val - C_val*D_coeff) / (A_coeff*E_coeff - B_coeff*D_coeff);
    
    % --- E. POINT P (Trajectory) ---
    RO4O2 = L1*exp(1i*offset);
    L_P = 0.065;
    R_P = RO4O2 + L_P * exp(1i * (q4_Brown + offset));
    
    % --- F. STORE DATA ---
    History_Angles(i,:) = rad2deg([q2_Green, q3, q4_Grey, q2_Cyan, q3_Red, q3_Blue, q4_Brown] + offset);
    History_Velocities(i,:) = [w_Green, w_Yellow, w_Grey, w_Cyan, w_Red, w_Blue, w_Brown];
    History_Accelerations(i,:) = [alpha_Green, alpha_Yellow, alpha_Grey, alpha_Cyan, alpha_Red, alpha_Blue, alpha_Brown];
    History_P(i,:) = [real(R_P), imag(R_P)];
end

% ==========================================
% 4. PLOTTING RESULTS
% ==========================================
link_names = {'Green', 'Yellow', 'Grey', 'Cyan', 'Red', 'Blue', 'Brown'};
colors = {'g', 'y', 'k', 'c', 'r', 'b', [0.6 0.3 0]};

% 1) Plot Angles
figure(1); hold on; grid on;
for k=1:7
    plot(time, History_Angles(:,k), 'DisplayName', link_names{k}, 'Color', colors{k}, 'LineWidth', 1.5);
end
title('1) Angular Positions vs Time'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Location','best');

% 2) Plot Angular Velocities
figure(2); hold on; grid on;
for k=1:7
    plot(time, History_Velocities(:,k), 'DisplayName', link_names{k}, 'Color', colors{k}, 'LineWidth', 1.5);
end
title('2) Angular Velocities vs Time'); xlabel('Time (s)'); ylabel('Velocity (rad/s)');
legend('Location','best');

% 3) Plot Angular Accelerations
figure(3); hold on; grid on;
for k=1:7
    plot(time, History_Accelerations(:,k), 'DisplayName', link_names{k}, 'Color', colors{k}, 'LineWidth', 1.5);
end
title('3) Angular Accelerations vs Time'); xlabel('Time (s)'); ylabel('Acceleration (rad/s^2)');
legend('Location','best');

% 4) Plot Trajectory of Point P
figure(4); hold on; grid on; axis equal;
plot(History_P(:,1), History_P(:,2), 'm-', 'LineWidth', 2);
plot(History_P(1,1), History_P(1,2), 'mo', 'MarkerFaceColor', 'g'); % Start
plot(History_P(end,1), History_P(end,2), 'mo', 'MarkerFaceColor', 'r'); % End
title('4) Trajectory of Point P'); xlabel('X (m)'); ylabel('Y (m)');
