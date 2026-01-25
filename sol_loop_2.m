clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d) - Pink
L2 = 0.118; % Crank (a) - Cyan
L3 = 0.210; % Coupler (b) - Red
L4 = 0.118; % Rocker (c) - Grey

a = L2;
b = L3;
c = L4;
d = L1;

% Ground Offset
offset_deg = 0.81;
offset = deg2rad(offset_deg);

% --- 2. Define K Constants (Inverse Analysis: Link 4 is Input) ---
% Swapping 'a' and 'c' roles 
K1 = d/c; 
K2 = d/a;
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% ==========================================
% CASE 1: Open Circuit (Input Theta 4 = -17.944)
% ==========================================
q4d_open = -17.944; 
q4_1 = deg2rad(q4d_open) - offset; 

% --- Theta 2 Coefficients ---
A1 = cos(q4_1) - K1 - K2*cos(q4_1) + K3;
B1 = -2*sin(q4_1);
C1 = K1 - (K2+1)*cos(q4_1) + K3;

% --- Theta 3 Coefficients ---
P1 = -2*b*(d + c*cos(q4_1));
Q1 = -2*b*c*sin(q4_1);
R1 = d^2 + c^2 + b^2 - a^2 + 2*d*c*cos(q4_1);

D1 = R1 - P1;
E1 = 2*Q1;
F1 = R1 + P1;

% --- Solve Theta 2 & 3 (Case 1) ---
% Using Solution 2 (Minus root) to force Link 2 DOWN
q2_open = 2*atan((-B1 - sqrt(B1^2 - 4*A1*C1))/(2*A1)); 
q3_open = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1)); 

q2_open_d = rad2deg(q2_open) + offset_deg;
q3_open_d = rad2deg(q3_open) + offset_deg;

% ==========================================
% CASE 2: Crossed Circuit (Input Theta 4 = -92.2333)
% ==========================================
q4d_cross = -92.2333; 
q4_2 = deg2rad(q4d_cross) - offset;

% --- Theta 2 Coefficients ---
A2 = cos(q4_2) - K1 - K2*cos(q4_2) + K3;
B2 = -2*sin(q4_2);
C2 = K1 - (K2+1)*cos(q4_2) + K3;

% --- Theta 3 Coefficients ---
P2 = -2*b*(d + c*cos(q4_2));
Q2 = -2*b*c*sin(q4_2);
R2 = d^2 + c^2 + b^2 - a^2 + 2*d*c*cos(q4_2);

D2 = R2 - P2;
E2 = 2*Q2;
F2 = R2 + P2;

% --- Solve Theta 2 & 3 (Case 2) ---
% Using Solution 2 (Minus root) to force Link 2 DOWN
q2_cross = 2*atan((-B2 - sqrt(B2^2 - 4*A2*C2))/(2*A2)); 
q3_cross = 2*atan((-E2 - sqrt(E2^2 - 4*D2*F2))/(2*D2)); 

q2_cross_d = rad2deg(q2_cross) + offset_deg;
q3_cross_d = rad2deg(q3_cross) + offset_deg;


% ==========================================
% Vector Calculation & Plotting
% ==========================================
RO4O2 = d*exp(j*offset); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Vector Case 1 (Open) ---
RA1 = a*exp(j*(q2_open + offset));       
RBA1 = b*exp(j*(q3_open + offset));        
RBO4_1 = c*exp(j*(q4_1 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1); 
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Vector Case 2 (Crossed) ---
RA2 = a*exp(j*(q2_cross + offset));       
RBA2 = b*exp(j*(q3_cross + offset));
RBO4_2 = c*exp(j*(q4_2 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2); 
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% --- Plotting Case 1 ---
figure(1)
title('Case 1: Open Circuit (Input q4 = -17.944)');
hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); % Pink
quiver(0,0, RA1x, RA1y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Cyan
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Red
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); % Grey
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% --- Plotting Case 2 ---
figure(2)
title('Case 2: Crossed Circuit (Input q4 = -92.2333)');
hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); % Pink
quiver(0,0, RA2x, RA2y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Cyan
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Red
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); % Grey
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Case 1 (Open Input): Theta 2 = ', num2str(q2_open_d), ', Theta 3 = ', num2str(q3_open_d)]);
disp(['Case 2 (Crossed Input): Theta 2 = ', num2str(q2_cross_d), ', Theta 3 = ', num2str(q3_cross_d)]);
