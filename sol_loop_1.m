clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Link 1 (d, Ground/Pink)
L2 = 0.180; % Link 2 (a, Crank/Green)
L3 = 0.180; % Link 3 (b, Coupler/Yellow)
L4 = 0.118; % Link 4 (c, Rocker/Grey)

a = L2;
b = L3;
c = L4;
d = L1;

% --- 2. Input Parameter (Theta 3 Known) ---
% Ground (Link 1) is lifted by 0.81 degrees
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q3d_global = 19.94; % Known Theta 3 (Global Input)
q3 = deg2rad(q3d_global) - offset; % Convert to Local Theta 3

% --- 3. Define K Constants for INVERSE Analysis ---
% Swap roles: Link 3 (b) acts as Input to find others.
% To find Theta 4 (Rocker), use Inverse K constants:
K1 = d/b; 
K2 = d/c;
K3 = (b^2 - a^2 + c^2 + d^2)/(2*b*c); 

% To find Theta 2 (Crank), use Inverse K constants:
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
% Two solutions: q41 and q42
q41 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A)); 
q42 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));

% Convert to Global Degrees
q41d = rad2deg(q41) + offset_deg;
q42d = rad2deg(q42) + offset_deg;

% --- 7. Solve for Theta 2 (Crank) ---
% Two solutions: q21 and q22
q21 = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));
q22 = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D));

% Convert to Global Degrees
q21d = rad2deg(q21) + offset_deg;
q22d = rad2deg(q22) + offset_deg;

% --- 8. Vector Calculation for Plotting ---
% Ground Vector (Same for both)
RO4O2 = d*exp(j*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Case 1: Open Circuit Configuration ---
% Pairing q21 and q41 (Assumption for Open config based on assembly)
RA1 = a*exp(j*(q21 + offset));       
RBA = b*exp(j*(q3 + offset));        
RBO4_1 = c*exp(j*(q41 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBAx = real(RBA); RBAy = imag(RBA); % Note: RBA is same vector direction
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Case 2: Crossed Circuit Configuration ---
% Pairing q22 and q42
RA2 = a*exp(j*(q22 + offset));       
RBO4_2 = c*exp(j*(q42 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% --- 9. Plotting Case 1 (Open Circuit) ---
figure(1)
title('Case 1: Open Circuit');
hold on;
% Ground (Pink/Black)
quiver(0,0, RO4O2x, RO4O2y, 0, 'black', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Crank (Green/Red)
quiver(0,0, RA1x, RA1y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
% Coupler (Yellow/Blue)
quiver(RA1x, RA1y, RBAx, RBAy, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
% Rocker (Grey/Green)
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'green', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% --- 10. Plotting Case 2 (Crossed Circuit) ---
figure(2)
title('Case 2: Crossed Circuit');
hold on;
% Ground
quiver(0,0, RO4O2x, RO4O2y, 0, 'black', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Crank
quiver(0,0, RA2x, RA2y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
% Coupler
quiver(RA2x, RA2y, RBAx, RBAy, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
% Rocker
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'green', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Input Theta 3 (Global): ', num2str(q3d_global)]);
disp(['Case 1 (Open) - Theta 2: ', num2str(q21d), '  Theta 4: ', num2str(q41d)]);
disp(['Case 2 (Crossed) - Theta 2: ', num2str(q22d), '  Theta 4: ', num2str(q42d)]);
