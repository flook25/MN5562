clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d) - Pink
L2 = 0.118; % Crank (a) - Blue (Half)
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

q4d_global = -92.2333; % Known Theta 4 (Global Input)
q4 = deg2rad(q4d_global) - offset; % Local Theta 4

% --- 3. Find Theta 2 (Crank) ---
% Using Freudenstein Equation adapted for Theta 4 Input
% K constants (Standard definition)
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);

% Coefficients for Theta 2: A*t^2 + B*t + C = 0
% Derived from: (K1 - cos(q4))cos(q2) - sin(q4)sin(q2) + (K3 - K2*cos(q4)) = 0
term1 = K1 - cos(q4);
term2 = -sin(q4);
term3 = K3 - K2*cos(q4);

% Quadratic Coefficients
A = term3 - term1;
B = 2*term2;
C = term3 + term1;

% Solve for Theta 2 (Two Solutions)
q21 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A)); 
q22 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));

q21d = rad2deg(q21) + offset_deg;
q22d = rad2deg(q22) + offset_deg;

% --- 4. Find Theta 3 (Coupler - Red Link) ---
% Derived from Vector Loop Geometry: R2 + R3 = R1 + R4
% We eliminate Theta 2 to solve for Theta 3 directly
% Form: P*cos(q3) + Q*sin(q3) + R = 0
P = -2*b*(d + c*cos(q4));
Q = -2*b*c*sin(q4);
R = d^2 + c^2 + b^2 - a^2 + 2*d*c*cos(q4);

% Quadratic Coefficients (D*t^2 + E*t + F = 0)
D = R - P;
E = 2*Q;
F = R + P;

% Solve for Theta 3 (Two Solutions)
q31 = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D)); 
q32 = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));

q31d = rad2deg(q31) + offset_deg;
q32d = rad2deg(q32) + offset_deg;

% --- 5. Vector Calculation for Plotting ---
% Ground Vector
RO4O2 = d*exp(j*offset); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Case 1: Open Circuit (Using set 1) ---
RA1 = a*exp(j*(q21 + offset));       
RBA1 = b*exp(j*(q31 + offset));        
RBO4_1 = c*exp(j*(q4 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1); 
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% --- Case 2: Crossed Circuit (Using set 2) ---
RA2 = a*exp(j*(q22 + offset));       
RBA2 = b*exp(j*(q32 + offset));
RBO4_2 = c*exp(j*(q4 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% --- 6. Plotting ---

% Plot Case 1
figure(1)
title('Case 1: Open Circuit');
hold on;
% Ground (L1 - Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (L2 - Blue)
quiver(0,0, RA1x, RA1y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (L3 - Red)
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (L4 - Grey) - From O4 to B
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Plot Case 2
figure(2)
title('Case 2: Crossed Circuit');
hold on;
% Ground (L1 - Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (L2 - Blue)
quiver(0,0, RA2x, RA2y, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (L3 - Red)
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (L4 - Grey)
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Input Theta 4 (Global): ', num2str(q4d_global)]);
disp(['Case 1 - Theta 2: ', num2str(q21d), '  Theta 3: ', num2str(q31d)]);
disp(['Case 2 - Theta 2: ', num2str(q22d), '  Theta 3: ', num2str(q32d)]);
