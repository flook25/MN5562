clear all
close all
clc

% --- 1. Define Parameters ---
L1 = 0.210; % Ground (d)
L2 = 0.118; % Crank (a) - Half blue
L3 = 0.210; % Coupler (b) - Red
L4 = 0.118; % Rocker (c) - Grey

a = L2;
b = L3;
c = L4;
d = L1;

% --- 2. Input Parameter (Theta 4 Known) ---
% Ground is lifted by 0.81 degrees
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q4_global_deg = -92.2333; % Known Input Angle
q4 = deg2rad(q4_global_deg) - offset; % Local Theta 4

% --- 3. Find Theta 2 (Crank) ---
% Define Standard K Constants
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);

% Coefficients for Theta 2 (Derived from Freudenstein)
A_q2 = (K1 + 1)*cos(q4) + K2 + K3;
B_q2 = -2*sin(q4);
C_q2 = (K1 - 1)*cos(q4) - K2 + K3;

% Solve for Theta 2 (Two solutions)
q2_1 = 2*atan((-B_q2 + sqrt(B_q2^2 - 4*A_q2*C_q2))/(2*A_q2)); 
q2_2 = 2*atan((-B_q2 - sqrt(B_q2^2 - 4*A_q2*C_q2))/(2*A_q2));

% --- 4. Find Theta 3 (Coupler) ---
% Coefficients derived from Geometric Loop closure
P = 2*b*(c*cos(q4) + d);
Q = 2*b*c*sin(q4);
R = c^2 + d^2 + b^2 - a^2 + 2*c*d*cos(q4);

% Coefficients for quadratic (D*t^2 + E*t + F = 0)
D_q3 = R - P;
E_q3 = 2*Q;
F_q3 = R + P;

% Solve for Theta 3 (Two solutions)
% Note: Pairing q3_1 with q2_1 typically represents one assembly mode
q3_1 = 2*atan((-E_q3 + sqrt(E_q3^2 - 4*D_q3*F_q3))/(2*D_q3));
q3_2 = 2*atan((-E_q3 - sqrt(E_q3^2 - 4*D_q3*F_q3))/(2*D_q3));

% --- 5. Convert to Global Angles ---
q21d = rad2deg(q2_1) + offset_deg;
q22d = rad2deg(q2_2) + offset_deg;

q31d = rad2deg(q3_1) + offset_deg;
q32d = rad2deg(q3_2) + offset_deg;

% --- 6. Vector Calculation for Plotting ---
% Ground Vector
RO4O2 = d*exp(j*offset); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Case 1: Open Circuit (Configuration 1) ---
% Using q2_1 and q3_1
RA1 = a*exp(j*(q2_1 + offset));       
RBA1 = b*exp(j*(q3_1 + offset));        
RBO4_1 = c*exp(j*(q4 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1); 
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);
RB1x = RA1x + RBA1x; RB1y = RA1y + RBA1y; % Check Point B

% --- Case 2: Crossed Circuit (Configuration 2) ---
% Using q2_2 and q3_2
RA2 = a*exp(j*(q2_2 + offset));       
RBA2 = b*exp(j*(q3_2 + offset));
RBO4_2 = c*exp(j*(q4 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);
RB2x = RA2x + RBA2x; RB2y = RA2y + RBA2y;

% --- 7. Plotting (Using quiver only) ---

% Plot Case 1
figure(1)
title('Case 1: Open Circuit');
hold on;
% Ground (Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Blue - Link 2)
quiver(0,0, RA1x, RA1y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Red - Link 3)
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Grey - Link 4) - Drawn from Ground End (O4)
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Plot Case 2
figure(2)
title('Case 2: Crossed Circuit');
hold on;
% Ground (Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Blue - Link 2)
quiver(0,0, RA2x, RA2y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Red - Link 3)
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Grey - Link 4)
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Input Theta 4 (Global): ', num2str(q4_global_deg)]);
disp(['Case 1 - Theta 2: ', num2str(q21d), '  Theta 3: ', num2str(q31d)]);
disp(['Case 2 - Theta 2: ', num2str(q22d), '  Theta 3: ', num2str(q32d)]);
