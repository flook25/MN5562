clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % d (Ground)
L2 = 0.180; % a (Crank)
L3 = 0.180; % b (Coupler)
L4 = 0.118; % c (Rocker)

a = L2;
b = L3;
c = L4;
d = L1;

% --- 2. Input Parameter ---
% Ground Angle (Theta 1) is 0.81 degrees (Fixed)
q1_rad = deg2rad(0.81); 

% Known Theta 3 (Coupler Angle) = 19.94 degrees (Global angle)
q3_global = 19.94;
q3 = deg2rad(q3_global); 

% --- 3. Calculation for Theta 4 (Rocker) ---
% Vector Loop: R2 + R3 - R4 - R1 = 0
% Rearrange to eliminate Theta 2: R2 = R1 + R4 - R3
% Real: a*cos(q2) = d*cos(q1) + c*cos(q4) - b*cos(q3)
% Imag: a*sin(q2) = d*sin(q1) + c*sin(q4) - b*sin(q3)
% Square and add to eliminate q2, resulting in form: P*cos(q4) + Q*sin(q4) + R = 0

% Terms for Theta 4 Equation (Constants based on knowns q1, q3)
K1_inv = d/b; 
K2_inv = d/c; 
K3_inv = (b^2 - a^2 + c^2 + d^2)/(2*b*c);

% Derived Coefficients for solving Theta 4 directly with q1 offset included:
% P4*cos(q4) + Q4*sin(q4) + R4 = 0
% P4 = 2*c*(d*cos(q1) - b*cos(q3));
% Q4 = 2*c*(d*sin(q1) - b*sin(q3));
% R4 = d^2 + b^2 + c^2 - a^2 - 2*b*d*cos(q1 - q3); -- (Using cosine rule expansion)

% Let's stick to the simplest algebraic expansion to ensure correctness without custom functions
% Constant Terms combining knowns (d, q1, b, q3)
Px = d*cos(q1) - b*cos(q3);
Py = d*sin(q1) - b*sin(q3);
% Equation: a^2 = (Px + c*cos(q4))^2 + (Py + c*sin(q4))^2
% a^2 = Px^2 + 2*Px*c*cos(q4) + c^2*cos^2(q4) + Py^2 + 2*Py*c*sin(q4) + c^2*sin^2(q4)
% a^2 = Px^2 + Py^2 + c^2 + 2*c*Px*cos(q4) + 2*c*Py*sin(q4)

A_4 = 2*c*Px;           % Coeff of cos(q4)
B_4 = 2*c*Py;           % Coeff of sin(q4)
C_4 = Px^2 + Py^2 + c^2 - a^2; % Constant term

% Convert to Quadratic form for t = tan(q4/2)
% A_quad * t^2 + B_quad * t + C_quad = 0
% cos(q4) = (1-t^2)/(1+t^2), sin(q4) = 2t/(1+t^2)
% A_4(1-t^2) + B_4(2t) + C_4(1+t^2) = 0
% (C_4 - A_4)*t^2 + (2*B_4)*t + (C_4 + A_4) = 0

A_sol4 = C_4 - A_4;
B_sol4 = 2*B_4;
C_sol4 = C_4 + A_4;

% Solve Theta 4 (Two Solutions)
q41 = 2*atan((-B_sol4 + sqrt(B_sol4^2 - 4*A_sol4*C_sol4))/(2*A_sol4));
q42 = 2*atan((-B_sol4 - sqrt(B_sol4^2 - 4*A_sol4*C_sol4))/(2*A_sol4));

q41d = rad2deg(q41);
q42d = rad2deg(q42);

% --- 4. Calculation for Theta 2 (Crank) ---
% Rearrange to eliminate Theta 4: R4 = R2 + R3 - R1
% Real: c*cos(q4) = a*cos(q2) + b*cos(q3) - d*cos(q1)
% Imag: c*sin(q4) = a*sin(q2) + b*sin(q3) - d*sin(q1)
% Square and add...
% Equation form: P2*cos(q2) + Q2*sin(q2) + R2 = 0

% Constant terms combining knowns (b, q3, d, q1)
Mx = b*cos(q3) - d*cos(q1);
My = b*sin(q3) - d*sin(q1);
% c^2 = (a*cos(q2) + Mx)^2 + (a*sin(q2) + My)^2
% c^2 = a^2 + Mx^2 + My^2 + 2*a*Mx*cos(q2) + 2*a*My*sin(q2)

A_2 = 2*a*Mx; % Coeff of cos(q2)
B_2 = 2*a*My; % Coeff of sin(q2)
C_2 = a^2 + Mx^2 + My^2 - c^2;

% Convert to Quadratic form for t = tan(q2/2)
% (C_2 - A_2)*t^2 + (2*B_2)*t + (C_2 + A_2) = 0
A_sol2 = C_2 - A_2;
B_sol2 = 2*B_2;
C_sol2 = C_2 + A_2;

% Solve Theta 2 (Two Solutions)
q21 = 2*atan((-B_sol2 - sqrt(B_sol2^2 - 4*A_sol2*C_sol2))/(2*A_sol2));
q22 = 2*atan((-B_sol2 + sqrt(B_sol2^2 - 4*A_sol2*C_sol2))/(2*A_sol2));

q21d = rad2deg(q21);
q22d = rad2deg(q22);

% --- 5. Vector Calculation for Plotting ---
% Ground Vector (Starts at 0,0 ends at d*e^jq1)
RO2 = 0; % Origin
RO4 = d*exp(j*q1_rad); % Ground End Point

% Vectors
% Config 1 (Open Circuit) - Pairing q21 and q41
RA1 = a*exp(j*q21);
RB1 = RA1 + b*exp(j*q3); % Calculated from Crank side
RB1_check = RO4 + c*exp(j*q41); % Check from Rocker side

% Config 2 (Crossed Circuit) - Pairing q22 and q42
RA2 = a*exp(j*q22);
RB2 = RA2 + b*exp(j*q3);
RB2_check = RO4 + c*exp(j*q42);

% --- 6. Plotting ---

% Figure 1: Open Circuit (Configuration 1)
figure(1)
hold on;
% Ground (Link 1 - Pink/Magenta)
plot([real(RO2), real(RO4)], [imag(RO2), imag(RO4)], 'm-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'm');
% Crank (Link 2 - Green)
plot([real(RO2), real(RA1)], [imag(RO2), imag(RA1)], 'g-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'g');
% Coupler (Link 3 - Yellow)
plot([real(RA1), real(RB1)], [imag(RA1), imag(RB1)], 'y-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'y');
% Rocker (Link 4 - Grey)
plot([real(RO4), real(RB1)], [imag(RO4), imag(RB1)], 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'Marker', 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.5 0.5 0.5]);

axis equal; grid on;
title('Configuration 1 (Open Circuit)');
xlabel('x (m)'); ylabel('y (m)');
% Force X-axis to be horizontal reference
yline(0, '--k', 'Alpha', 0.3); 

% Figure 2: Crossed Circuit (Configuration 2)
figure(2)
hold on;
% Ground (Link 1 - Pink/Magenta)
plot([real(RO2), real(RO4)], [imag(RO2), imag(RO4)], 'm-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'm');
% Crank (Link 2 - Green)
plot([real(RO2), real(RA2)], [imag(RO2), imag(RA2)], 'g-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'g');
% Coupler (Link 3 - Yellow)
plot([real(RA2), real(RB2)], [imag(RA2), imag(RB2)], 'y-o', 'LineWidth', 3, 'MarkerSize', 8, 'MarkerFaceColor', 'y');
% Rocker (Link 4 - Grey)
plot([real(RO4), real(RB2)], [imag(RO4), imag(RB2)], 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'Marker', 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.5 0.5 0.5]);

axis equal; grid on;
title('Configuration 2 (Crossed Circuit)');
xlabel('x (m)'); ylabel('y (m)');
yline(0, '--k', 'Alpha', 0.3);

% Display Values
disp('--- Calculated Angles (Global Coordinates) ---')
disp(['Input Theta 3: ', num2str(q3_global)]);
disp(['Ground Theta 1: ', num2str(rad2deg(q1_rad))]);
disp(' ');
disp('--- Configuration 1 (Open) ---');
disp(['Theta 2 (Crank): ', num2str(q21d)]);
disp(['Theta 4 (Rocker): ', num2str(q41d)]);
disp(' ');
disp('--- Configuration 2 (Crossed) ---');
disp(['Theta 2 (Crank): ', num2str(q22d)]);
disp(['Theta 4 (Rocker): ', num2str(q42d)]);
