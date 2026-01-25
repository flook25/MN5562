clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d) - Pink
L2 = 0.118; % Crank (a) - Cyan (Input)
L3 = 0.210; % Coupler (b) - Blue
L4 = 0.118; % Rocker (c) - Brown

a = L2;
b = L3;
c = L4;
d = L1;

% Ground Offset
offset_deg = 0.81;
offset = deg2rad(offset_deg);

% --- 2. Define K Constants (Forward Analysis: Theta 2 is Input) ---
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% ==========================================
% CASE 1: Check Input -92.2333 (User suspects Open)
% ==========================================
q2d_case1 = -92.2333;
q2_1 = deg2rad(q2d_case1) - offset;

% --- Calculate Coefficients for Theta 4 ---
A1 = cos(q2_1) - K1 - K2*cos(q2_1) + K3;
B1 = -2*sin(q2_1);
C1 = K1 - (K2+1)*cos(q2_1) + K3;

% --- Calculate Coefficients for Theta 3 ---
D1 = cos(q2_1) - K1 + K4*cos(q2_1) + K5;
E1 = -2*sin(q2_1);
F1 = K1 + (K4-1)*cos(q2_1) + K5;

% --- Solve (Using Minus Sqrt for OPEN Config) ---
q4_case1 = 2*atan((-B1 - sqrt(B1^2 - 4*A1*C1))/(2*A1));
q3_case1 = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1));

q4_case1_d = rad2deg(q4_case1) + offset_deg;
q3_case1_d = rad2deg(q3_case1) + offset_deg;


% ==========================================
% CASE 2: Check Input -17.944
% ==========================================
q2d_case2 = -17.944;
q2_2 = deg2rad(q2d_case2) - offset;

% --- Calculate Coefficients for Theta 4 ---
A2 = cos(q2_2) - K1 - K2*cos(q2_2) + K3;
B2 = -2*sin(q2_2);
C2 = K1 - (K2+1)*cos(q2_2) + K3;

% --- Calculate Coefficients for Theta 3 ---
D2 = cos(q2_2) - K1 + K4*cos(q2_2) + K5;
E2 = -2*sin(q2_2);
F2 = K1 + (K4-1)*cos(q2_2) + K5;

% --- Solve (Using Plus Sqrt for CROSSED Config) ---
q4_case2 = 2*atan((-B2 + sqrt(B2^2 - 4*A2*C2))/(2*A2));
q3_case2 = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));

q4_case2_d = rad2deg(q4_case2) + offset_deg;
q3_case2_d = rad2deg(q3_case2) + offset_deg;


% ==========================================
% Vector Calculation & Plotting
% ==========================================
RO4O2 = d*exp(j*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Vector Case 1 (-92.2333 as Open) ---
RA1 = a*exp(j*(q2_1 + offset));
RBA1 = b*exp(j*(q3_case1 + offset));
RBO4_1 = c*exp(j*(q4_case1 + offset));

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1);
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Vector Case 2 (-17.944 as Crossed) ---
RA2 = a*exp(j*(q2_2 + offset));
RBA2 = b*exp(j*(q3_case2 + offset));
RBO4_2 = c*exp(j*(q4_case2 + offset));

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% --- Plotting Case 1 ---
figure(1)
title('Case 1: Input -92.2333 (Open Circuit)');
hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); % Pink
quiver(0,0, RA1x, RA1y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Cyan
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Blue
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.6 0.3 0], 'MaxHeadSize', 0.5, 'LineWidth', 3); % Brown
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% --- Plotting Case 2 ---
figure(2)
title('Case 2: Input -17.944 (Crossed Circuit)');
hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); % Pink
quiver(0,0, RA2x, RA2y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Cyan
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Blue
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.6 0.3 0], 'MaxHeadSize', 0.5, 'LineWidth', 3); % Brown
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Case 1 (Input -92.2333): Theta 3 = ', num2str(q3_case1_d), ', Theta 4 = ', num2str(q4_case1_d)]);
disp(['Case 2 (Input -17.944): Theta 3 = ', num2str(q3_case2_d), ', Theta 4 = ', num2str(q4_case2_d)]);
