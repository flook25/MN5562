clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (Pink)
L2 = 0.180; % Crank (Green)
L3 = 0.180; % Coupler (Yellow)
L4 = 0.118; % Rocker (Grey)

a = L2; b = L3; c = L4; d = L1;

% --- 2. Input Parameter (Position) ---
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q3d_global = 19.94; % Input Theta 3
q3 = deg2rad(q3d_global); % Global angle

% Local q3 for K calculation
q3_local = q3 - offset; 

% --- 3. Position Analysis (Inverse Kinematics) ---
% Finding Theta 4 (Rocker)
K1 = d/b; K2 = d/c; K3 = (b^2 - a^2 + c^2 + d^2)/(2*b*c); 
A_pos = cos(q3_local) - K1 - K2*cos(q3_local) + K3;
B_pos = -2*sin(q3_local);
C_pos = K1 - (K2+1)*cos(q3_local) + K3;

q4_loc1 = 2*atan((-B_pos - sqrt(B_pos^2 - 4*A_pos*C_pos))/(2*A_pos)); 
q4_loc2 = 2*atan((-B_pos + sqrt(B_pos^2 - 4*A_pos*C_pos))/(2*A_pos));

% Finding Theta 2 (Crank)
K4 = d/a; K5 = (c^2 - d^2 - b^2 - a^2)/(2*b*a); 
D_pos = cos(q3_local) - K1 + K4*cos(q3_local) + K5;
E_pos = -2*sin(q3_local);
F_pos = K1 + (K4-1)*cos(q3_local) + K5;

q2_loc1 = 2*atan((-E_pos - sqrt(E_pos^2 - 4*D_pos*F_pos))/(2*D_pos)); 
q2_loc2 = 2*atan((-E_pos + sqrt(E_pos^2 - 4*D_pos*F_pos))/(2*D_pos));

% Global Angles
q21 = q2_loc1 + offset; q41 = q4_loc1 + offset; % Case 1 (Open)
q22 = q2_loc2 + offset; q42 = q4_loc2 + offset; % Case 2 (Crossed)

% --- 4. Velocity Analysis (Inverse: Given w3, find w2 & w4) ---
w3 = -2.2; % rad/s

% Case 1 (Open)
w2_1 = (b * w3 * sin(q3 - q41)) / (a * sin(q41 - q21));
w4_1 = (b * w3 * sin(q3 - q21)) / (c * sin(q41 - q21));

% Case 2 (Crossed)
w2_2 = (b * w3 * sin(q3 - q42)) / (a * sin(q42 - q22));
w4_2 = (b * w3 * sin(q3 - q22)) / (c * sin(q42 - q22));

% --- 5. Acceleration Analysis (Inverse: Given alpha3, find alpha2 & alpha4) ---
alpha3 = 0.003; % rad/s^2

% --- Case 1 Calculation ---
% Solve System: A*alpha2 + B*alpha4 = C
%               D*alpha2 + E*alpha4 = F
% Derived from Real/Imag parts of Acceleration Loop Equation

A1 = -a*sin(q21);
B1 = c*sin(q41);
% RHS Real Part (Move knowns to right)
C1 = a*w2_1^2*cos(q21) + b*alpha3*sin(q3) + b*w3^2*cos(q3) - c*w4_1^2*cos(q41);

D1 = a*cos(q21);
E1 = -c*cos(q41);
% RHS Imag Part
F1 = a*w2_1^2*sin(q21) - b*alpha3*cos(q3) + b*w3^2*sin(q3) - c*w4_1^2*sin(q41);

% Solve using Cramer's Rule logic
Det1 = A1*E1 - B1*D1;
alpha2_1 = (C1*E1 - B1*F1) / Det1;
alpha4_1 = (A1*F1 - C1*D1) / Det1;

% Vector Construction Case 1
% AA = An + At
AA_n1 = -a * w2_1^2 * exp(1j*q21);
AA_t1 = 1j * a * alpha2_1 * exp(1j*q21);
AA_1 = AA_n1 + AA_t1;

% ABA = An + At
ABA_n1 = -b * w3^2 * exp(1j*q3);
ABA_t1 = 1j * b * alpha3 * exp(1j*q3);
ABA_1 = ABA_n1 + ABA_t1;

% AB = AA + ABA
AB_1 = AA_1 + ABA_1;

% AB Check from Link 4 side
AB_n4_1 = -c * w4_1^2 * exp(1j*q41);
AB_t4_1 = 1j * c * alpha4_1 * exp(1j*q41);
AB_Check1 = AB_n4_1 + AB_t4_1;


% --- Case 2 Calculation ---
A2 = -a*sin(q22);
B2 = c*sin(q42);
C2 = a*w2_2^2*cos(q22) + b*alpha3*sin(q3) + b*w3^2*cos(q3) - c*w4_2^2*cos(q42);

D2 = a*cos(q22);
E2 = -c*cos(q42);
F2 = a*w2_2^2*sin(q22) - b*alpha3*cos(q3) + b*w3^2*sin(q3) - c*w4_2^2*sin(q42);

Det2 = A2*E2 - B2*D2;
alpha2_2 = (C2*E2 - B2*F2) / Det2;
alpha4_2 = (A2*F2 - C2*D2) / Det2;

% Vector Construction Case 2
AA_n2 = -a * w2_2^2 * exp(1j*q22);
AA_t2 = 1j * a * alpha2_2 * exp(1j*q22);
AA_2 = AA_n2 + AA_t2;

ABA_n2 = -b * w3^2 * exp(1j*q3);
ABA_t2 = 1j * b * alpha3 * exp(1j*q3);
ABA_2 = ABA_n2 + ABA_t2;

AB_2 = AA_2 + ABA_2;

AB_n4_2 = -c * w4_2^2 * exp(1j*q42);
AB_t4_2 = 1j * c * alpha4_2 * exp(1j*q42);
AB_Check2 = AB_n4_2 + AB_t4_2;

% --- 6. Plotting ---
% Scale for Acceleration Vectors (Adjust to fit graph)
s_acc = 1/5; 

% Position Vectors for drawing links
RO4O2 = d*exp(1j*offset); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

RA1 = a*exp(1j*q21); RB1 = RA1 + b*exp(1j*q3);
RA2 = a*exp(1j*q22); RB2 = RA2 + b*exp(1j*q3);

% Extract Components for Plotting Case 1
AA_1x = real(AA_1); AA_1y = imag(AA_1);
ABA_1x = real(ABA_1); ABA_1y = imag(ABA_1);
AB_1x = real(AB_1); AB_1y = imag(AB_1);
AB_Check1x = real(AB_Check1); AB_Check1y = imag(AB_Check1);

% Extract Components for Plotting Case 2
AA_2x = real(AA_2); AA_2y = imag(AA_2);
ABA_2x = real(ABA_2); ABA_2y = imag(ABA_2);
AB_2x = real(AB_2); AB_2y = imag(AB_2);
AB_Check2x = real(AB_Check2); AB_Check2y = imag(AB_Check2);


% --- Figure 1: Open Circuit ---
figure(1)
title('Case 1: Open Circuit (Acceleration)'); hold on;

% Draw Links using Quiver (Tail -> Head)
% Ground (Pink)
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'LineWidth', 4, 'MaxHeadSize', 0.1);
% Crank (Green)
quiver(0,0, real(RA1), imag(RA1), 0, 'Color', 'g', 'LineWidth', 3, 'MaxHeadSize', 0.1);
% Coupler (Yellow) - From A to B
quiver(real(RA1), imag(RA1), real(RB1)-real(RA1), imag(RB1)-imag(RA1), 0, 'Color', 'y', 'LineWidth', 3, 'MaxHeadSize', 0.1);
% Rocker (Grey) - From O4 to B
quiver(RO4O2x, RO4O2y, real(RB1)-RO4O2x, imag(RB1)-RO4O2y, 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.1);

% Draw Acceleration Vectors
% Red = AA
quiver(real(RA1), imag(RA1), AA_1x*s_acc, AA_1y*s_acc, 0, 'red', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Blue = ABA (Relative Accel at B)
quiver(real(RB1), imag(RB1), ABA_1x*s_acc, ABA_1y*s_acc, 0, 'blue', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Green = AB (Total Accel at B)
quiver(real(RB1), imag(RB1), AB_1x*s_acc, AB_1y*s_acc, 0, 'green', 'LineWidth', 2, 'MaxHeadSize', 0.5);
% Black = Check AB (From Link 4)
quiver(real(RB1), imag(RB1), AB_Check1x*s_acc, AB_Check1y*s_acc, 0, 'black', 'LineWidth', 1, 'MaxHeadSize', 0.5);

axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');

% --- Figure 2: Crossed Circuit ---
figure(2)
title('Case 2: Crossed Circuit (Acceleration)'); hold on;

% Draw Links
% Ground
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'LineWidth', 4, 'MaxHeadSize', 0.1);
% Crank
quiver(0,0, real(RA2), imag(RA2), 0, 'Color', 'g', 'LineWidth', 3, 'MaxHeadSize', 0.1);
% Coupler
quiver(real(RA2), imag(RA2), real(RB2)-real(RA2), imag(RB2)-imag(RA2), 0, 'Color', 'y', 'LineWidth', 3, 'MaxHeadSize', 0.1);
% Rocker
quiver(RO4O2x, RO4O2y, real(RB2)-RO4O2x, imag(RB2)-RO4O2y, 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 3, 'MaxHeadSize', 0.1);

% Draw Acceleration Vectors
quiver(real(RA2), imag(RA2), AA_2x*s_acc, AA_2y*s_acc, 0, 'red', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(RB2), imag(RB2), ABA_2x*s_acc, ABA_2y*s_acc, 0, 'blue', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(RB2), imag(RB2), AB_2x*s_acc, AB_2y*s_acc, 0, 'green', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(RB2), imag(RB2), AB_Check2x*s_acc, AB_Check2y*s_acc, 0, 'black', 'LineWidth', 1, 'MaxHeadSize', 0.5);

axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');


% Display Results
disp('--- Acceleration Results ---');
disp(['Input alpha3: ', num2str(alpha3)]);
disp(' ');
disp('Case 1 (Open):');
disp(['  alpha2: ', num2str(alpha2_1), ' rad/s^2']);
disp(['  alpha4: ', num2str(alpha4_1), ' rad/s^2']);
disp(' ');
disp('Case 2 (Crossed):');
disp(['  alpha2: ', num2str(alpha2_2), ' rad/s^2']);
disp(['  alpha4: ', num2str(alpha4_2), ' rad/s^2']);
