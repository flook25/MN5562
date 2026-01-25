clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d) - Pink
L2 = 0.118; % Crank (a) - Cyan (Input from prev loop)
L3 = 0.210; % Coupler (b) - Blue
L4 = 0.118; % Rocker (c) - Brown/Orange

a = L2;
b = L3;
c = L4;
d = L1;

% Ground Offset
offset_deg = 0.81;
offset = deg2rad(offset_deg);

% --- 2. Define K Constants (Forward Analysis: Link 2 is Input) ---
% Standard Freudenstein for Theta 2 Input -> Find Theta 4 & 3
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% ==========================================
% CASE 1: Open Circuit (Input Theta 2 = -17.944)
% ==========================================
q2d_open = -17.944;
q2_1 = deg2rad(q2d_open) - offset; % Local Theta 2

% --- Calculate Coefficients for Theta 4 (Rocker) ---
A1 = cos(q2_1) - K1 - K2*cos(q2_1) + K3;
B1 = -2*sin(q2_1);
C1 = K1 - (K2+1)*cos(q2_1) + K3;

% --- Calculate Coefficients for Theta 3 (Coupler) ---
D1 = cos(q2_1) - K1 + K4*cos(q2_1) + K5;
E1 = -2*sin(q2_1);
F1 = K1 + (K4-1)*cos(q2_1) + K5;

% --- Solve for Theta 4 & 3 (Case 1) ---
% Using standard quadratic formula (+/- sqrt)
% We select the root that maintains the Open assembly form
q4_open = 2*atan((-B1 - sqrt(B1^2 - 4*A1*C1))/(2*A1));
q3_open = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1));

q4_open_d = rad2deg(q4_open) + offset_deg;
q3_open_d = rad2deg(q3_open) + offset_deg;


% ==========================================
% CASE 2: Crossed Circuit (Input Theta 2 = -92.2333)
% ==========================================
q2d_cross = -92.2333;
q2_2 = deg2rad(q2d_cross) - offset; % Local Theta 2

% --- Calculate Coefficients for Theta 4 ---
A2 = cos(q2_2) - K1 - K2*cos(q2_2) + K3;
B2 = -2*sin(q2_2);
C2 = K1 - (K2+1)*cos(q2_2) + K3;

% --- Calculate Coefficients for Theta 3 ---
D2 = cos(q2_2) - K1 + K4*cos(q2_2) + K5;
E2 = -2*sin(q2_2);
F2 = K1 + (K4-1)*cos(q2_2) + K5;

% --- Solve for Theta 4 & 3 (Case 2) ---
% Using standard quadratic formula
q4_cross = 2*atan((-B2 + sqrt(B2^2 - 4*A2*C2))/(2*A2));
q3_cross = 2*atan((-E2 + sqrt(E2^2 - 4*D2*F2))/(2*D2));

q4_cross_d = rad2deg(q4_cross) + offset_deg;
q3_cross_d = rad2deg(q3_cross) + offset_deg;


% ==========================================
% Vector Calculation & Plotting
% ==========================================
% Ground Vector
RO4O2 = d*exp(j*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Vector Case 1 (Open) ---
RA1 = a*exp(j*(q2_1 + offset));
RBA1 = b*exp(j*(q3_open + offset));
RBO4_1 = c*exp(j*(q4_open + offset));

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1);
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Vector Case 2 (Crossed) ---
RA2 = a*exp(j*(q2_2 + offset));
RBA2 = b*exp(j*(q3_cross + offset));
RBO4_2 = c*exp(j*(q4_cross + offset));

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% --- Plotting Case 1 ---
figure(1)
title('Case 1: Open Circuit (Input Theta 2 = -17.944)');
hold on;
% Ground (Link 1 - Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4);
% Crank (Link 2 - Cyan)
quiver(0,0, RA1x, RA1y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3);
% Coupler (Link 3 - Blue)
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3);
% Rocker (Link 4 - Brown/Orange)
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.85 0.5 0.1], 'MaxHeadSize', 0.5, 'LineWidth', 3);
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% --- Plotting Case 2 ---
figure(2)
title('Case 2: Crossed Circuit (Input Theta 2 = -92.2333)');
hold on;
% Ground (Link 1 - Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4);
% Crank (Link 2 - Cyan)
quiver(0,0, RA2x, RA2y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3);
% Coupler (Link 3 - Blue)
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3);
% Rocker (Link 4 - Brown/Orange)
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.85 0.5 0.1], 'MaxHeadSize', 0.5, 'LineWidth', 3);
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Case 1 (Open): Input q2=',num2str(q2d_open),' -> Calc q3=', num2str(q3_open_d), ', q4=', num2str(q4_open_d)]);
disp(['Case 2 (Crossed): Input q2=',num2str(q2d_cross),' -> Calc q3=', num2str(q3_cross_d), ', q4=', num2str(q4_cross_d)]);
