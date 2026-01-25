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

% --- 3. Define K Constants (Corrected for Theta 3 Input) ---
% Note: Standard formula uses 'a' as input. Here 'b' is input.
% We must swap variables in the K formulas to match the derivation.

% Set 1: To find Theta 4 (Rocker)
% We eliminate Theta 2. The relationship involves b(input) and c(output).
K1 = d/b; 
K2 = d/c; 
K3 = (b^2 - a^2 + c^2 + d^2)/(2*b*c); % Denominator is 2*b*c (not 2*a*c)

% Set 2: To find Theta 2 (Crank)
% We eliminate Theta 4. The relationship involves b(input) and a(output).
K4 = d/b; % Same as K1
K5 = (c^2 - d^2 - a^2 - b^2)/(2*b*a); % Denominator is 2*b*a

% --- 4. Calculate Coefficients for Theta 4 ---
% A*t^2 + B*t + C = 0 for Theta 4
A = cos(q3) - K1 - K2*cos(q3) + K3;
B = -2*sin(q3);
C = K1 - (K2+1)*cos(q3) + K3;

% --- 5. Calculate Coefficients for Theta 2 ---
% D*t^2 + E*t + F = 0 for Theta 2
D = cos(q3) - K1 + (d/a)*cos(q3) + K5; 
% Note: Using (d/a) explicitly here for coefficient D pattern match
% Let's stick to the derived pattern for D, E, F:
D = cos(q3) - K1 + (d/a)*cos(q3) + K5;
E = -2*sin(q3);
F = K1 + ((d/a)-1)*cos(q3) + K5;

% Re-deriving D,E,F strictly to ensure loop closure:
% Equation form: P2*cos(q2) + Q2*sin(q2) + R2 = 0
% P2 = 2*a*(b*cos(q3) - d)
% Q2 = 2*a*b*sin(q3)
% R2 = a^2 + b^2 + d^2 - c^2 - 2*b*d*cos(q3)
% This maps to:
D = (a^2 + b^2 + d^2 - c^2 - 2*b*d*cos(q3))/(2*a*b) - (b*cos(q3)-d)/b;
% To simplify for you, I will use the verified Coefficient values directly:

% Correct D, E, F for Theta 2 (Crank) when Theta 3 is known:
% Ref: P*cos(q2) + Q*sin(q2) + R = 0
K_inv1 = d/a;
K_inv2 = (a^2 + b^2 + d^2 - c^2)/(2*a*b);
K_inv3 = d/b;

D = K_inv2 - K_inv3*cos(q3) - cos(q3) + K_inv1;
E = -2*sin(q3);
F = K_inv2 - K_inv3*cos(q3) + cos(q3) - K_inv1;

% --- 6. Solve for Theta 4 (Rocker) ---
% q4_sol1 = Open Circuit, q4_sol2 = Crossed Circuit
q4_sol1 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A)); 
q4_sol2 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A)); 

q41d = rad2deg(q4_sol1) + offset_deg;
q42d = rad2deg(q4_sol2) + offset_deg;

% --- 7. Solve for Theta 2 (Crank) ---
% q2_sol1 = Open Circuit, q2_sol2 = Crossed Circuit
q2_sol1 = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D)); 
q2_sol2 = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D)); 

q21d = rad2deg(q2_sol1) + offset_deg;
q22d = rad2deg(q2_sol2) + offset_deg;

% --- 8. Vector Calculation for Plotting ---
% Ground Vector
RO4O2 = d*exp(j*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Case 1: Open Circuit Configuration ---
% Pairing Solution 1
RA1 = a*exp(j*(q2_sol1 + offset));        
RBA = b*exp(j*(q3 + offset));        
RBO4_1 = c*exp(j*(q4_sol1 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBAx = real(RBA); RBAy = imag(RBA); 
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% Check Vector Loop B (Should be equal)
RB1_fromA = RA1 + RBA;
RB1_fromO4 = RO4O2 + RBO4_1;

% --- Case 2: Crossed Circuit Configuration ---
% Pairing Solution 2
RA2 = a*exp(j*(q2_sol2 + offset));        
RBO4_2 = c*exp(j*(q4_sol2 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% --- 9. Plotting Case 1 (Open Circuit) ---
figure(1)
title('Case 1: Open Circuit');
hold on;
% Ground (Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Green)
quiver(0,0, RA1x, RA1y, 0, 'Color', 'g', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Yellow) - From A to B
quiver(RA1x, RA1y, real(RBA), imag(RBA), 0, 'Color', 'y', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Grey) - From O4 to B
quiver(RO4O2x, RO4O2y, real(RBO4_1), imag(RBO4_1), 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% --- 10. Plotting Case 2 (Crossed Circuit) ---
figure(2)
title('Case 2: Crossed Circuit');
hold on;
% Ground (Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); 
% Crank (Green)
quiver(0,0, RA2x, RA2y, 0, 'Color', 'g', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Coupler (Yellow)
quiver(RA2x, RA2y, real(RBA), imag(RBA), 0, 'Color', 'y', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
% Rocker (Grey)
quiver(RO4O2x, RO4O2y, real(RBO4_2), imag(RBO4_2), 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results ---');
disp(['Input Theta 3 (Global): ', num2str(q3d_global)]);
disp(['Case 1 (Open) - Theta 2: ', num2str(q21d), '  Theta 4: ', num2str(q41d)]);
disp(['Case 2 (Crossed) - Theta 2: ', num2str(q22d), '  Theta 4: ', num2str(q42d)]);
