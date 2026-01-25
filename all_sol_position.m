clear all
close all
clc

% ==========================================
% SECTION 1: PARAMETERS & INPUT
% ==========================================
% --- Dimensions (Meters) ---
L1 = 0.210; % Ground (d) - Pink
L2_Loop1 = 0.180; % Crank Loop 1 (a) - Green
L3_Loop1 = 0.180; % Coupler Loop 1 (b) - Yellow
L4_Shared = 0.118; % Shared Rocker (c) - Grey

L2_Loop2 = 0.118; % Crank Loop 2 (Cyan)
L3_Loop2 = 0.210; % Coupler Loop 2 (Red)

L2_Loop3 = 0.118; % Crank Loop 3 (Cyan - Extended)
L3_Loop3 = 0.210; % Coupler Loop 3 (Blue)
L4_Loop3 = 0.118; % Rocker Loop 3 (Brown)

d = L1; % Common Ground

% --- Global Input ---
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q3d_global = 19.94; % Input Theta 3 (Yellow)
q3 = deg2rad(q3d_global) - offset;

% ==========================================
% SECTION 2: LOOP 1 (Green-Yellow-Grey)
% Input: Yellow (Theta 3)
% ==========================================
a = L2_Loop1; b = L3_Loop1; c = L4_Shared;

% K Constants (Input b)
K1 = d/b;
K2 = d/c;
K3 = (b^2 - a^2 + c^2 + d^2)/(2*b*c);
K4 = d/a;
K5 = (c^2 - d^2 - b^2 - a^2)/(2*b*a);

% Coefficients for Theta 4 (Grey)
A = cos(q3) - K1 - K2*cos(q3) + K3;
B = -2*sin(q3);
C = K1 - (K2+1)*cos(q3) + K3;

% Coefficients for Theta 2 (Green)
D = cos(q3) - K1 + K4*cos(q3) + K5;
E = -2*sin(q3);
F = K1 + (K4-1)*cos(q3) + K5;

% Solve Angles (Case 1: Open / Case 2: Crossed)
q4_L1_open = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_open = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));

q4_L1_cross = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_cross = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D));

% ==========================================
% SECTION 3: LOOP 2 (Cyan-Red-Grey)
% Input: Grey (Theta 4 from Loop 1)
% ==========================================
a = L2_Loop2; b = L3_Loop2; c = L4_Shared;

% K Constants (Input c - Grey)
K1_L2 = d/c;
K2_L2 = d/a;
K3_L2 = (c^2 - b^2 + a^2 + d^2)/(2*c*a);
K4_L2 = d/b;
K5_L2 = (a^2 - d^2 - c^2 - b^2)/(2*c*b);

% --- Case 1 Calculation (OPEN) ---
q_in_1 = q4_L1_open; 
% Coeffs Theta 2 (Cyan)
A_L2_1 = cos(q_in_1) - K1_L2 - K2_L2*cos(q_in_1) + K3_L2;
B_L2_1 = -2*sin(q_in_1);
C_L2_1 = K1_L2 - (K2_L2+1)*cos(q_in_1) + K3_L2;
% Coeffs Theta 3 (Red)
D_L2_1 = cos(q_in_1) - K1_L2 + K4_L2*cos(q_in_1) + K5_L2;
E_L2_1 = -2*sin(q_in_1);
F_L2_1 = K1_L2 + (K4_L2-1)*cos(q_in_1) + K5_L2;

% ** FIXED OPEN CASE: FLIPPED SQRT for Cyan & Red **
q2_Cyan_1 = 2*atan((-B_L2_1 + sqrt(B_L2_1^2 - 4*A_L2_1*C_L2_1))/(2*A_L2_1)); 
q3_Red_1  = 2*atan((-E_L2_1 + sqrt(E_L2_1^2 - 4*D_L2_1*F_L2_1))/(2*D_L2_1));

% --- Case 2 Calculation (CROSSED) ---
q_in_2 = q4_L1_cross;
% Coeffs Theta 2 (Cyan)
A_L2_2 = cos(q_in_2) - K1_L2 - K2_L2*cos(q_in_2) + K3_L2;
B_L2_2 = -2*sin(q_in_2);
C_L2_2 = K1_L2 - (K2_L2+1)*cos(q_in_2) + K3_L2;
% Coeffs Theta 3 (Red)
D_L2_2 = cos(q_in_2) - K1_L2 + K4_L2*cos(q_in_2) + K5_L2;
E_L2_2 = -2*sin(q_in_2);
F_L2_2 = K1_L2 + (K4_L2-1)*cos(q_in_2) + K5_L2;

% Solve (Standard Crossed)
q2_Cyan_2 = 2*atan((-B_L2_2 + sqrt(B_L2_2^2 - 4*A_L2_2*C_L2_2))/(2*A_L2_2)); 
q3_Red_2  = 2*atan((-E_L2_2 + sqrt(E_L2_2^2 - 4*D_L2_2*F_L2_2))/(2*D_L2_2));

% ==========================================
% SECTION 4: LOOP 3 (Cyan-Blue-Brown)
% Input: Cyan from Loop 2 (+180 deg for Straight Line)
% ==========================================
a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;

% K Constants (Input a - Cyan)
K1_L3 = d/a;
K2_L3 = d/c;
K3_L3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4_L3 = d/b;
K5_L3 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- Case 1 Calculation (OPEN) ---
% FORCE STRAIGHT LINE: Add pi (180 deg) to Cyan angle from Loop 2
q_in_L3_1 = q2_Cyan_1 + pi; 

% Coeffs Theta 4 (Brown)
A_L3_1 = cos(q_in_L3_1) - K1_L3 - K2_L3*cos(q_in_L3_1) + K3_L3;
B_L3_1 = -2*sin(q_in_L3_1);
C_L3_1 = K1_L3 - (K2_L3+1)*cos(q_in_L3_1) + K3_L3;
% Coeffs Theta 3 (Blue)
D_L3_1 = cos(q_in_L3_1) - K1_L3 + K4_L3*cos(q_in_L3_1) + K5_L3;
E_L3_1 = -2*sin(q_in_L3_1);
F_L3_1 = K1_L3 + (K4_L3-1)*cos(q_in_L3_1) + K5_L3;

% ** FIXED OPEN CASE: FLIPPED SQRT for Brown & Blue **
q4_Brown_1 = 2*atan((-B_L3_1 + sqrt(B_L3_1^2 - 4*A_L3_1*C_L3_1))/(2*A_L3_1));
q3_Blue_1  = 2*atan((-E_L3_1 + sqrt(E_L3_1^2 - 4*D_L3_1*F_L3_1))/(2*D_L3_1));

% --- Case 2 Calculation (CROSSED) ---
q_in_L3_2 = q2_Cyan_2 + pi; % Force Straight Line

% Coeffs Theta 4 (Brown)
A_L3_2 = cos(q_in_L3_2) - K1_L3 - K2_L3*cos(q_in_L3_2) + K3_L3;
B_L3_2 = -2*sin(q_in_L3_2);
C_L3_2 = K1_L3 - (K2_L3+1)*cos(q_in_L3_2) + K3_L3;
% Coeffs Theta 3 (Blue)
D_L3_2 = cos(q_in_L3_2) - K1_L3 + K4_L3*cos(q_in_L3_2) + K5_L3;
E_L3_2 = -2*sin(q_in_L3_2);
F_L3_2 = K1_L3 + (K4_L3-1)*cos(q_in_L3_2) + K5_L3;

% Solve (Standard Crossed)
q4_Brown_2 = 2*atan((-B_L3_2 + sqrt(B_L3_2^2 - 4*A_L3_2*C_L3_2))/(2*A_L3_2));
q3_Blue_2  = 2*atan((-E_L3_2 + sqrt(E_L3_2^2 - 4*D_L3_2*F_L3_2))/(2*D_L3_2));

% ==========================================
% SECTION 5: VECTORS & PLOTTING
% ==========================================
RO4O2 = d*exp(j*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Calculate Vectors Case 1 ---
V_Green_1 = L2_Loop1 * exp(j*(q2_L1_open + offset));
V_Yellow_1 = L3_Loop1 * exp(j*(q3 + offset));
V_Grey_1 = L4_Shared * exp(j*(q4_L1_open + offset));

V_Cyan_Down_1 = L2_Loop2 * exp(j*(q2_Cyan_1 + offset));
V_Red_1 = L3_Loop2 * exp(j*(q3_Red_1 + offset));

V_Cyan_Up_1 = L2_Loop3 * exp(j*(q_in_L3_1 + offset));
V_Blue_1 = L3_Loop3 * exp(j*(q3_Blue_1 + offset));
V_Brown_1 = L4_Loop3 * exp(j*(q4_Brown_1 + offset));

% --- Calculate Vectors Case 2 ---
V_Green_2 = L2_Loop1 * exp(j*(q2_L1_cross + offset));
V_Yellow_2 = L3_Loop1 * exp(j*(q3 + offset));
V_Grey_2 = L4_Shared * exp(j*(q4_L1_cross + offset));

V_Cyan_Down_2 = L2_Loop2 * exp(j*(q2_Cyan_2 + offset));
V_Red_2 = L3_Loop2 * exp(j*(q3_Red_2 + offset));

V_Cyan_Up_2 = L2_Loop3 * exp(j*(q_in_L3_2 + offset));
V_Blue_2 = L3_Loop3 * exp(j*(q3_Blue_2 + offset));
V_Brown_2 = L4_Loop3 * exp(j*(q4_Brown_2 + offset));

% --- Plot Case 1 ---
figure(1)
title('Case 1: Open Circuit'); hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'LineWidth', 4, 'MaxHeadSize', 0.5); % Pink
% Loop 1
quiver(0,0, real(V_Green_1), imag(V_Green_1), 0, 'green', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(real(V_Green_1), imag(V_Green_1), real(V_Yellow_1), imag(V_Yellow_1), 0, 'yellow', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(V_Grey_1), imag(V_Grey_1), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Loop 2
quiver(0,0, real(V_Cyan_Down_1), imag(V_Cyan_Down_1), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(real(V_Cyan_Down_1), imag(V_Cyan_Down_1), real(V_Red_1), imag(V_Red_1), 0, 'red', 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Loop 3
quiver(0,0, real(V_Cyan_Up_1), imag(V_Cyan_Up_1), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'LineStyle', ':');
quiver(real(V_Cyan_Up_1), imag(V_Cyan_Up_1), real(V_Blue_1), imag(V_Blue_1), 0, 'blue', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(V_Brown_1), imag(V_Brown_1), 0, 'Color', [0.85 0.5 0.1], 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;

% --- Plot Case 2 ---
figure(2)
title('Case 2: Crossed Circuit'); hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'LineWidth', 4, 'MaxHeadSize', 0.5); % Pink
% Loop 1
quiver(0,0, real(V_Green_2), imag(V_Green_2), 0, 'green', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(real(V_Green_2), imag(V_Green_2), real(V_Yellow_2), imag(V_Yellow_2), 0, 'yellow', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(V_Grey_2), imag(V_Grey_2), 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Loop 2
quiver(0,0, real(V_Cyan_Down_2), imag(V_Cyan_Down_2), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(real(V_Cyan_Down_2), imag(V_Cyan_Down_2), real(V_Red_2), imag(V_Red_2), 0, 'red', 'LineWidth', 3, 'MaxHeadSize', 0.5);
% Loop 3
quiver(0,0, real(V_Cyan_Up_2), imag(V_Cyan_Up_2), 0, 'cyan', 'LineWidth', 3, 'MaxHeadSize', 0.5, 'LineStyle', ':');
quiver(real(V_Cyan_Up_2), imag(V_Cyan_Up_2), real(V_Blue_2), imag(V_Blue_2), 0, 'blue', 'LineWidth', 3, 'MaxHeadSize', 0.5);
quiver(RO4O2x, RO4O2y, real(V_Brown_2), imag(V_Brown_2), 0, 'Color', [0.85 0.5 0.1], 'LineWidth', 3, 'MaxHeadSize', 0.5);
axis equal; grid on;

% Results
disp('--- Results ---');
disp(['Case 1 Green: ', num2str(rad2deg(q2_L1_open)+offset_deg)]);
disp(['Case 1 Cyan(Up): ', num2str(rad2deg(q_in_L3_1)+offset_deg)]);
