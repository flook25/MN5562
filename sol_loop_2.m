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

% --- 2. Input Parameter (Theta 4 Known) ---
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q4d_global = -92.2333; % Known Input Theta 4
q4 = deg2rad(q4d_global) - offset; % Local Theta 4

% --- 3. Define K Constants (Swapped a and c for Inverse) ---
% Link 4 (c) is Input, Link 2 (a) is Output
K1 = d/c; 
K2 = d/a;
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a);
K4 = d/b;
K5 = (a^2 - d^2 - c^2 - b^2)/(2*c*b);

% --- 4. Calculate Coefficients for Theta 2 (Crank) ---
% Pattern: A, B, C used to find q2 (Output)
A = cos(q4) - K1 - K2*cos(q4) + K3;
B = -2*sin(q4);
C = K1 - (K2+1)*cos(q4) + K3;

% --- 5. Calculate Coefficients for Theta 3 (Coupler) ---
% Pattern: D, E, F used to find q3 (Coupler)
D = cos(q4) - K1 + K4*cos(q4) + K5;
E = -2*sin(q4);
F = K1 + (K4-1)*cos(q4) + K5;

% --- 6. Solve for Theta 2 (Crank) ---
% Two solutions using standard pattern
q2_1 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A)); 
q2_2 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));

q21d = rad2deg(q2_1) + offset_deg;
q22d = rad2deg(q2_2) + offset_deg;

% --- 7. Solve for Theta 3 (Coupler) ---
% Two solutions using standard pattern
q3_1 = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D)); 
q3_2 = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));

q31d = rad2deg(q3_1) + offset_deg;
q32d = rad2deg(q3_2) + offset_deg;

% --- 8. Vector Calculation for Plotting ---
% Ground Vector
RO4O2 = d*exp(j*offset); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Case 1: Open Circuit (Using pair 1) ---
RA1 = a*exp(j*(q2_1 + offset));       
RBA1 = b*exp(j*(q3_1 + offset));        
RBO4_1 = c*exp(j*(q4 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1); 
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Case 2: Crossed Circuit (Using pair 2) ---
RA2 = a*exp(j*(q2_2 + offset));       
RBA2 = b*exp(j*(q3_2 + offset));
RBO4_2 = c*exp(j*(q4 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2); 
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% --- 9. Plotting Case 1 (Open Circuit) ---
figure(1)
title('Case 1: Open Circuit');
hold on;
% Ground (Link 1 - Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Link 2 - Cyan)
quiver(0,0, RA1x, RA1y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Link 3 - Red)
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Link 4 - Grey)
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% --- 10. Plotting Case 2 (Crossed Circuit) ---
figure(2)
title('Case 2: Crossed Circuit');
hold on;
% Ground (Link 1 - Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Link 2 - Cyan)
quiver(0,0, RA2x, RA2y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Link 3 - Red)
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Link 4 - Grey)
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Input Theta 4 (Global): ', num2str(q4d_global)]);
disp(['Case 1 - Theta 2: ', num2str(q21d), '  Theta 3: ', num2str(q31d)]);
disp(['Case 2 - Theta 2: ', num2str(q22d), '  Theta 3: ', num2str(q32d)]);
