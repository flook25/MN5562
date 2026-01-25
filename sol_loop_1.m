clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Length of Link 1 (d, Ground/Pink)
L2 = 0.180; % Length of Link 2 (a, Crank/Green)
L3 = 0.180; % Length of Link 3 (b, Coupler/Yellow)
L4 = 0.118; % Length of Link 4 (c, Rocker/Grey)

a = L2;
b = L3;
c = L4;
d = L1;

% --- 2. Input Parameter (Theta 3 Known) ---
% Ground is lifted by 0.81 degrees
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q3d_global = 19.94; % Known Theta 3 (Global)
q3 = deg2rad(q3d_global) - offset; % Local Theta 3 for calculation

% --- 3. Define K Constants for INVERSE Analysis ---
% Standard K uses d/a, d/c... 
% Here we swap roles: Input is Link 3 (b).
% To find Theta 4 (Rocker), we use 'Inverse' K1, K2, K3
K1 = d/b; 
K2 = d/c;
K3 = (b^2 - a^2 + c^2 + d^2)/(2*b*c); % Notice b and a are swapped in numer/denom

% To find Theta 2 (Crank), we use 'Inverse' K4, K5
K4 = d/a; 
K5 = (c^2 - d^2 - b^2 - a^2)/(2*b*a); % Notice b and a are swapped

% --- 4. Calculate Coefficients for Theta 4 ---
A = cos(q3) - K1 - K2*cos(q3) + K3;
B = -2*sin(q3);
C = K1 - (K2+1)*cos(q3) + K3;

% --- 5. Calculate Coefficients for Theta 2 ---
D = cos(q3) - K1 + K4*cos(q3) + K5;
E = -2*sin(q3);
F = K1 + (K4-1)*cos(q3) + K5;

% --- 6. Solve for Theta 4 (Rocker) ---
% Two solutions: q41 (Config 1) and q42 (Config 2)
q41 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A)); 
q42 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));

% Convert to Global Degrees
q41d = rad2deg(q41) + offset_deg;
q42d = rad2deg(q42) + offset_deg;

% --- 7. Solve for Theta 2 (Crank) ---
% Two solutions: q21 (Config 1) and q22 (Config 2)
q21 = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));
q22 = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D));

% Convert to Global Degrees
q21d = rad2deg(q21) + offset_deg;
q22d = rad2deg(q22) + offset_deg;

% --- 8. Vector Calculation (Config 1: Open/Crossed A) ---
% Using q21 and q41 pair
RA1 = a*exp(j*(q21 + offset));       % Crank Vector
RBA = b*exp(j*(q3 + offset));        % Coupler Vector (Known Input)
RBO4_1 = c*exp(j*(q41 + offset));    % Rocker Vector
RO4O2 = d*exp(j*offset);             % Ground Vector

RB1 = RA1 + RBA; % Position B

% Extract Components Config 1
RA1x = real(RA1); RA1y = imag(RA1);
RBAx = real(RBA); RBAy = imag(RBA);
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);
RB1x = real(RB1); RB1y = imag(RB1);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- 9. Vector Calculation (Config 2: Open/Crossed B) ---
% Using q22 and q42 pair
RA2 = a*exp(j*(q22 + offset));       % Crank Vector
RBO4_2 = c*exp(j*(q42 + offset));    % Rocker Vector

RB2 = RA2 + RBA; % Position B

% Extract Components Config 2
RA2x = real(RA2); RA2y = imag(RA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);
RB2x = real(RB2); RB2y = imag(RB2);

% --- 10. Plotting ---
% Plotting Config 1
quiver(0,0, RO4O2x, RO4O2y, 0, 'black', 'MaxHeadSize', 0.5, 'LineWidth', 2); hold on; % Ground
quiver(0,0, RA1x, RA1y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2); % Crank
quiver(RA1x, RA1y, RBAx, RBAy, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 2); % Coupler
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'green', 'MaxHeadSize', 0.5, 'LineWidth', 2); % Rocker

% Plotting Config 2 (Dashed or lighter to distinguish)
quiver(0,0, RA2x, RA2y, 0, 'magenta', 'MaxHeadSize', 0.5, 'LineWidth', 1, 'LineStyle', '--'); % Crank 2
quiver(RA2x, RA2y, RBAx, RBAy, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 1, 'LineStyle', '--'); % Coupler 2 (Same pos, diff start)
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'yellow', 'MaxHeadSize', 0.5, 'LineWidth', 1, 'LineStyle', '--'); % Rocker 2

axis equal;
grid on;
title('Inverse Kinematics: Given Theta 3, Find Theta 2 & 4');
