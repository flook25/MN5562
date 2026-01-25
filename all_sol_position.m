clear all
close all
clc

% ==========================================
% LOOP 1: Green-Grey-Yellow
% Input: Theta 3 (Yellow) = 19.94
% Output: Theta 2 (Green), Theta 4 (Grey)
% ==========================================

% --- 1. Parameters ---
L1 = 0.210; % d
L2 = 0.180; % a (Green)
L3 = 0.180; % b (Yellow)
L4 = 0.118; % c (Grey)
a = L2; b = L3; c = L4; d = L1;

offset = deg2rad(0.81);
q3d_global = 19.94;
q3 = deg2rad(q3d_global) - offset;

% --- 2. K Constants (Inverse: b is Input) ---
K1 = d/b;
K2 = d/c;
K3 = (b^2 - a^2 + c^2 + d^2)/(2*b*c);
K4 = d/a;
K5 = (c^2 - d^2 - b^2 - a^2)/(2*b*a);

% --- 3. Coefficients for Theta 4 (Grey) ---
A = cos(q3) - K1 - K2*cos(q3) + K3;
B = -2*sin(q3);
C = K1 - (K2+1)*cos(q3) + K3;

% --- 4. Coefficients for Theta 2 (Green) ---
D = cos(q3) - K1 + K4*cos(q3) + K5;
E = -2*sin(q3);
F = K1 + (K4-1)*cos(q3) + K5;

% --- 5. Solve Angles (Loop 1) ---
% Case 1 (Open): Minus Sqrt for q4, Minus Sqrt for q2
q4_L1_open = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_open = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));

% Case 2 (Crossed): Plus Sqrt for q4, Plus Sqrt for q2
q4_L1_cross = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_cross = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D));

% --- 6. Vectors Loop 1 ---
RO4O2 = d*exp(j*offset);

% Open
RA_L1_open = a*exp(j*(q2_L1_open + offset));
RBA_L1_open = b*exp(j*(q3 + offset));
RBO4_L1_open = c*exp(j*(q4_L1_open + offset));

% Crossed
RA_L1_cross = a*exp(j*(q2_L1_cross + offset));
RBA_L1_cross = b*exp(j*(q3 + offset));
RBO4_L1_cross = c*exp(j*(q4_L1_cross + offset));


% ==========================================
% LOOP 2: Cyan-Red-Grey
% Input: Theta 4 (Grey from Loop 1)
% Output: Theta 2 (Cyan Down), Theta 3 (Red)
% ==========================================

% --- 1. Parameters ---
L2_new = 0.118; % a (Cyan)
L3_new = 0.210; % b (Red)
L4_new = 0.118; % c (Grey - Input)
a = L2_new; b = L3_new; c = L4_new; d = L1;

% --- 2. K Constants (Inverse: Link 4/c is Input) ---
K1 = d/c; 
K2 = d/a;
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 
K4 = d/b;
K5 = (a^2 - d^2 - c^2 - b^2)/(2*c*b);

% --- 3. Case 1 Open (Input from L1 Open) ---
q_in_open = q4_L1_open;

% Coeffs for Theta 2 (Cyan)
A_L2_open = cos(q_in_open) - K1 - K2*cos(q_in_open) + K3;
B_L2_open = -2*sin(q_in_open);
C_L2_open = K1 - (K2+1)*cos(q_in_open) + K3;

% Coeffs for Theta 3 (Red)
D_L2_open = cos(q_in_open) - K1 + K4*cos(q_in_open) + K5;
E_L2_open = -2*sin(q_in_open);
F_L2_open = K1 + (K4-1)*cos(q_in_open) + K5;

% Solve (Use Minus Sqrt to force Cyan DOWN)
q2_L2_open = 2*atan((-B_L2_open - sqrt(B_L2_open^2 - 4*A_L2_open*C_L2_open))/(2*A_L2_open));
q3_L2_open = 2*atan((-E_L2_open - sqrt(E_L2_open^2 - 4*D_L2_open*F_L2_open))/(2*D_L2_open));

% --- 4. Case 2 Crossed (Input from L1 Crossed) ---
q_in_cross = q4_L1_cross;

% Coeffs for Theta 2 (Cyan)
A_L2_cross = cos(q_in_cross) - K1 - K2*cos(q_in_cross) + K3;
B_L2_cross = -2*sin(q_in_cross);
C_L2_cross = K1 - (K2+1)*cos(q_in_cross) + K3;

% Coeffs for Theta 3 (Red)
D_L2_cross = cos(q_in_cross) - K1 + K4*cos(q_in_cross) + K5;
E_L2_cross = -2*sin(q_in_cross);
F_L2_cross = K1 + (K4-1)*cos(q_in_cross) + K5;

% Solve (Use Minus Sqrt to force Cyan DOWN)
q2_L2_cross = 2*atan((-B_L2_cross - sqrt(B_L2_cross^2 - 4*A_L2_cross*C_L2_cross))/(2*A_L2_cross));
q3_L2_cross = 2*atan((-E_L2_cross - sqrt(E_L2_cross^2 - 4*D_L2_cross*F_L2_cross))/(2*D_L2_cross));

% --- 5. Vectors Loop 2 ---
% Open
RA_L2_open = a*exp(j*(q2_L2_open + offset));
RBA_L2_open = b*exp(j*(q3_L2_open + offset));
RBO4_L2_open = c*exp(j*(q_in_open + offset));

% Crossed
RA_L2_cross = a*exp(j*(q2_L2_cross + offset));
RBA_L2_cross = b*exp(j*(q3_L2_cross + offset));
RBO4_L2_cross = c*exp(j*(q_in_cross + offset));


% ==========================================
% LOOP 3: Cyan-Blue-Brown
% Input: Theta 2 (Cyan from Loop 2 but UP)
% Output: Theta 3 (Blue), Theta 4 (Brown)
% ==========================================

% --- 1. Parameters ---
L2_L3 = 0.118; % a (Cyan Input)
L3_L3 = 0.210; % b (Blue)
L4_L3 = 0.118; % c (Brown)
a = L2_L3; b = L3_L3; c = L4_L3; d = L1;

% --- 2. K Constants (Forward: Link 2/a is Input) ---
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% --- 3. Case 1 Open (Input from L2 Open * -1) ---
q_in_L3_open = -q2_L2_open; % Flip to point UP

% Coeffs for Theta 4 (Brown)
A_L3_open = cos(q_in_L3_open) - K1 - K2*cos(q_in_L3_open) + K3;
B_L3_open = -2*sin(q_in_L3_open);
C_L3_open = K1 - (K2+1)*cos(q_in_L3_open) + K3;

% Coeffs for Theta 3 (Blue)
D_L3_open = cos(q_in_L3_open) - K1 + K4*cos(q_in_L3_open) + K5;
E_L3_open = -2*sin(q_in_L3_open);
F_L3_open = K1 + (K4-1)*cos(q_in_L3_open) + K5;

% Solve (Minus Sqrt for Open)
q4_L3_open = 2*atan((-B_L3_open - sqrt(B_L3_open^2 - 4*A_L3_open*C_L3_open))/(2*A_L3_open));
q3_L3_open = 2*atan((-E_L3_open - sqrt(E_L3_open^2 - 4*D_L3_open*F_L3_open))/(2*D_L3_open));

% --- 4. Case 2 Crossed (Input from L2 Crossed * -1) ---
q_in_L3_cross = -q2_L2_cross; % Flip to point UP

% Coeffs for Theta 4 (Brown)
A_L3_cross = cos(q_in_L3_cross) - K1 - K2*cos(q_in_L3_cross) + K3;
B_L3_cross = -2*sin(q_in_L3_cross);
C_L3_cross = K1 - (K2+1)*cos(q_in_L3_cross) + K3;

% Coeffs for Theta 3 (Blue)
D_L3_cross = cos(q_in_L3_cross) - K1 + K4*cos(q_in_L3_cross) + K5;
E_L3_cross = -2*sin(q_in_L3_cross);
F_L3_cross = K1 + (K4-1)*cos(q_in_L3_cross) + K5;

% Solve (Plus Sqrt for Crossed)
q4_L3_cross = 2*atan((-B_L3_cross + sqrt(B_L3_cross^2 - 4*A_L3_cross*C_L3_cross))/(2*A_L3_cross));
q3_L3_cross = 2*atan((-E_L3_cross + sqrt(E_L3_cross^2 - 4*D_L3_cross*F_L3_cross))/(2*D_L3_cross));

% --- 5. Vectors Loop 3 ---
% Open
RA_L3_open = a*exp(j*(q_in_L3_open + offset));
RBA_L3_open = b*exp(j*(q3_L3_open + offset));
RBO4_L3_open = c*exp(j*(q4_L3_open + offset));

% Crossed
RA_L3_cross = a*exp(j*(q_in_L3_cross + offset));
RBA_L3_cross = b*exp(j*(q3_L3_cross + offset));
RBO4_L3_cross = c*exp(j*(q4_L3_cross + offset));


% ==========================================
% PLOTTING
% ==========================================
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Figure 1: Case 1 (Open Circuit) ---
figure(1)
title('Complete Mechanism: Case 1 (Open)'); hold on;
% Ground
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); % Pink

% Loop 1
quiver(0,0, real(RA_L1_open), imag(RA_L1_open), 0, 'green', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Green
quiver(real(RA_L1_open), imag(RA_L1_open), real(RBA_L1_open), imag(RBA_L1_open), 0, 'yellow', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Yellow
quiver(RO4O2x, RO4O2y, real(RBO4_L1_open), imag(RBO4_L1_open), 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); % Grey

% Loop 2 (Attached to Grey)
quiver(0,0, real(RA_L2_open), imag(RA_L2_open), 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Cyan (Down)
quiver(real(RA_L2_open), imag(RA_L2_open), real(RBA_L2_open), imag(RBA_L2_open), 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Red

% Loop 3 (Attached to Cyan Up - Logic Mirror)
quiver(0,0, real(RA_L3_open), imag(RA_L3_open), 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3, 'LineStyle', ':'); % Cyan (Up)
quiver(real(RA_L3_open), imag(RA_L3_open), real(RBA_L3_open), imag(RBA_L3_open), 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Blue
quiver(RO4O2x, RO4O2y, real(RBO4_L3_open), imag(RBO4_L3_open), 0, 'Color', [0.85 0.5 0.1], 'MaxHeadSize', 0.5, 'LineWidth', 3); % Brown

axis equal; grid on;

% --- Figure 2: Case 2 (Crossed Circuit) ---
figure(2)
title('Complete Mechanism: Case 2 (Crossed)'); hold on;
% Ground
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); % Pink

% Loop 1
quiver(0,0, real(RA_L1_cross), imag(RA_L1_cross), 0, 'green', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
quiver(real(RA_L1_cross), imag(RA_L1_cross), real(RBA_L1_cross), imag(RBA_L1_cross), 0, 'yellow', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
quiver(RO4O2x, RO4O2y, real(RBO4_L1_cross), imag(RBO4_L1_cross), 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); 

% Loop 2
quiver(0,0, real(RA_L2_cross), imag(RA_L2_cross), 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
quiver(real(RA_L2_cross), imag(RA_L2_cross), real(RBA_L2_cross), imag(RBA_L2_cross), 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); 

% Loop 3
quiver(0,0, real(RA_L3_cross), imag(RA_L3_cross), 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3, 'LineStyle', ':'); 
quiver(real(RA_L3_cross), imag(RA_L3_cross), real(RBA_L3_cross), imag(RBA_L3_cross), 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 3); 
quiver(RO4O2x, RO4O2y, real(RBO4_L3_cross), imag(RBO4_L3_cross), 0, 'Color', [0.85 0.5 0.1], 'MaxHeadSize', 0.5, 'LineWidth', 3); 

axis equal; grid on;

% --- DISPLAY RESULTS ---
disp('--- FINAL ANGLE RESULTS (Degrees) ---');
disp(' ');
disp('CASE 1 (OPEN):');
disp(['  Green (L1 Crank): ', num2str(rad2deg(q2_L1_open) + offset_deg)]);
disp(['  Grey  (L1 Rocker): ', num2str(rad2deg(q4_L1_open) + offset_deg)]);
disp(['  Cyan  (L2 Crank): ', num2str(rad2deg(q2_L2_open) + offset_deg)]);
disp(['  Red   (L2 Coupler): ', num2str(rad2deg(q3_L2_open) + offset_deg)]);
disp(['  Blue  (L3 Coupler): ', num2str(rad2deg(q3_L3_open) + offset_deg)]);
disp(['  Brown (L3 Rocker): ', num2str(rad2deg(q4_L3_open) + offset_deg)]);
disp(' ');
disp('CASE 2 (CROSSED):');
disp(['  Green (L1 Crank): ', num2str(rad2deg(q2_L1_cross) + offset_deg)]);
disp(['  Grey  (L1 Rocker): ', num2str(rad2deg(q4_L1_cross) + offset_deg)]);
disp(['  Cyan  (L2 Crank): ', num2str(rad2deg(q2_L2_cross) + offset_deg)]);
disp(['  Red   (L2 Coupler): ', num2str(rad2deg(q3_L2_cross) + offset_deg)]);
disp(['  Blue  (L3 Coupler): ', num2str(rad2deg(q3_L3_cross) + offset_deg)]);
disp(['  Brown (L3 Rocker): ', num2str(rad2deg(q4_L3_cross) + offset_deg)]);
