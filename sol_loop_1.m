clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d) - Pink
L2 = 0.180; % Crank (a) - Green
L3 = 0.180; % Coupler (b) - Yellow
L4 = 0.118; % Rocker (c) - Grey

a = L2;
b = L3;
c = L4;
d = L1;

% --- 2. Input Parameter & Coordinate Transform ---
% Ground is lifted by 0.81 degrees
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q3d_global = 19.94; % Input Theta 3 (Global)
q3 = deg2rad(q3d_global) - offset; % Local Theta 3

% --- 3. Define K Constants for INVERSE Analysis ---
% Standard Norton uses 'a' as input. Here 'b' (Link 3) is input.
% We swap 'a' and 'b' in the formulas.

% Set 1: To find Theta 4 (Rocker)
K1 = d/b; 
K2 = d/c;
K3 = (b^2 - a^2 + c^2 + d^2)/(2*b*c); 

% Set 2: To find Theta 2 (Crank)
K4 = d/a; 
K5 = (c^2 - d^2 - b^2 - a^2)/(2*b*a); 

% --- 4. Calculate Coefficients for Theta 4 ---
A = cos(q3) - K1 - K2*cos(q3) + K3;
B = -2*sin(q3);
C = K1 - (K2+1)*cos(q3) + K3;

% --- 5. Calculate Coefficients for Theta 2 ---
D = cos(q3) - K1 + K4*cos(q3) + K5;
E = -2*sin(q3);
F = K1 + (K4-1)*cos(q3) + K5;

% --- 6. Solve for Theta 4 (Rocker) ---
% Using standard quadratic formula pattern
q4_sol1 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A)); % Open Circuit (typically)
q4_sol2 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A)); % Crossed Circuit (typically)

q41d = rad2deg(q4_sol1) + offset_deg;
q42d = rad2deg(q4_sol2) + offset_deg;

% --- 7. Solve for Theta 2 (Crank) ---
q2_sol1 = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D)); % Pair with q4_sol1
q2_sol2 = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D)); % Pair with q4_sol2

q21d = rad2deg(q2_sol1) + offset_deg;
q22d = rad2deg(q2_sol2) + offset_deg;

% --- 8. Vector Calculation for Plotting ---
% Ground Vector (Same for both)
RO4O2 = d*exp(j*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Case 1: Open Circuit Configuration ---
% Using Solution 1 pair
RA1 = a*exp(j*(q2_sol1 + offset));       
RBA = b*exp(j*(q3 + offset));        
RBO4_1 = c*exp(j*(q4_sol1 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBAx = real(RBA); RBAy = imag(RBA); 
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Case 2: Crossed Circuit Configuration ---
% Using Solution 2 pair
RA2 = a*exp(j*(q2_sol2 + offset));       
RBO4_2 = c*exp(j*(q4_sol2 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% --- 9. Plotting Case 1 (Open Circuit) ---
figure(1)
title('Case 1: Open Circuit');
hold on;
% Ground (L1 - Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (L2 - Green)
quiver(0,0, RA1x, RA1y, 0, 'Color', 'g', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (L3 - Yellow)
quiver(RA1x, RA1y, RBAx, RBAy, 0, 'Color', 'y', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (L4 - Grey)
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% --- 10. Plotting Case 2 (Crossed Circuit) ---
figure(2)
title('Case 2: Crossed Circuit');
hold on;
% Ground (L1 - Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (L2 - Green)
quiver(0,0, RA2x, RA2y, 0, 'Color', 'g', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (L3 - Yellow)
quiver(RA2x, RA2y, RBAx, RBAy, 0, 'Color', 'y', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (L4 - Grey)
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Input Theta 3 (Global): ', num2str(q3d_global)]);
disp(['Case 1 (Open) - Theta 2: ', num2str(q21d), '  Theta 4: ', num2str(q41d)]);
disp(['Case 2 (Crossed) - Theta 2: ', num2str(q22d), '  Theta 4: ', num2str(q42d)]);
