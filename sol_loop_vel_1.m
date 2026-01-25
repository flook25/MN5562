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
q3 = deg2rad(q3d_global); % Global angle used for velocity directly

% Calculate Local q3 for Position Analysis (K constants logic)
q3_local = q3 - offset; 

% --- 3. Position Analysis (Inverse Kinematics) ---
% Finding Theta 4 (Rocker)
K1 = d/b; K2 = d/c; K3 = (b^2 - a^2 + c^2 + d^2)/(2*b*c); 
A = cos(q3_local) - K1 - K2*cos(q3_local) + K3;
B = -2*sin(q3_local);
C = K1 - (K2+1)*cos(q3_local) + K3;

q4_loc1 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A)); 
q4_loc2 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A));

% Finding Theta 2 (Crank)
K4 = d/a; K5 = (c^2 - d^2 - b^2 - a^2)/(2*b*a); 
D = cos(q3_local) - K1 + K4*cos(q3_local) + K5;
E = -2*sin(q3_local);
F = K1 + (K4-1)*cos(q3_local) + K5;

q2_loc1 = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D)); 
q2_loc2 = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D));

% Global Angles (Adding Offset)
q21 = q2_loc1 + offset; q41 = q4_loc1 + offset; % Case 1 (Open)
q22 = q2_loc2 + offset; q42 = q4_loc2 + offset; % Case 2 (Crossed)

% --- 4. Velocity Analysis (Inverse: Given w3, find w2 & w4) ---
w3 = -2.2; % Given Input Velocity of Yellow Link (rad/s)

% --- Case 1: Open Circuit Velocities ---
% Derived Formulas:
w2_1 = (b * w3 * sin(q3 - q41)) / (a * sin(q41 - q21));
w4_1 = (b * w3 * sin(q3 - q21)) / (c * sin(q41 - q21));

% Vector Calculation (Case 1)
VA_1 = 1j * a * w2_1 * exp(1j * q21);
VBA_1 = 1j * b * w3 * exp(1j * q3);
VB_1 = VA_1 + VBA_1; % Resultant Velocity at B
VB_Check1 = 1j * c * w4_1 * exp(1j * q41); % Check from Link 4 side

% --- Case 2: Crossed Circuit Velocities ---
w2_2 = (b * w3 * sin(q3 - q42)) / (a * sin(q42 - q22));
w4_2 = (b * w3 * sin(q3 - q22)) / (c * sin(q42 - q22));

% Vector Calculation (Case 2)
VA_2 = 1j * a * w2_2 * exp(1j * q22);
VBA_2 = 1j * b * w3 * exp(1j * q3);
VB_2 = VA_2 + VBA_2; 
VB_Check2 = 1j * c * w4_2 * exp(1j * q42);

% --- 5. Extract Vector Components for Plotting ---
% Scale factor to make arrows visible but not too huge
scale = 1/15; 

% Components Case 1
VA_1x = real(VA_1); VA_1y = imag(VA_1);
VBA_1x = real(VBA_1); VBA_1y = imag(VBA_1);
VB_1x = real(VB_1); VB_1y = imag(VB_1);
VB_Check1x = real(VB_Check1); VB_Check1y = imag(VB_Check1);

% Components Case 2
VA_2x = real(VA_2); VA_2y = imag(VA_2);
VBA_2x = real(VBA_2); VBA_2y = imag(VBA_2);
VB_2x = real(VB_2); VB_2y = imag(VB_2);
VB_Check2x = real(VB_Check2); VB_Check2y = imag(VB_Check2);

% Position Vectors for placing the quiver arrows (From previous part)
RO4O2 = d*exp(1j*offset); RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);
RA1 = a*exp(1j*q21); RB1 = RA1 + b*exp(1j*q3);
RA2 = a*exp(1j*q22); RB2 = RA2 + b*exp(1j*q3);

% --- 6. Plotting ---

% Figure 1: Open Circuit
figure(1)
title('Case 1: Open Circuit (Velocity Vectors)'); hold on;
% Draw Linkage first (Background)
plot([0 real(RA1)],[0 imag(RA1)],'g-','LineWidth',2); % Crank
plot([real(RA1) real(RB1)],[imag(RA1) imag(RB1)],'y-','LineWidth',2); % Coupler
plot([real(RO4O2) real(RB1)],[imag(RO4O2) imag(RB1)],'Color',[0.5 0.5 0.5],'LineWidth',2); % Rocker
plot([0 real(RO4O2)],[0 imag(RO4O2)],'m-','LineWidth',2); % Ground

% Plot Velocity Vectors (Using your pattern)
% Red = VA (at Point A)
quiver(real(RA1), imag(RA1), VA_1x*scale, VA_1y*scale, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2);
% Blue = VBA (Relative Velocity at Point B)
quiver(real(RB1), imag(RB1), VBA_1x*scale, VBA_1y*scale, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 2);
% Green = VB (Resultant Velocity at Point B)
quiver(real(RB1), imag(RB1), VB_1x*scale, VB_1y*scale, 0, 'green', 'MaxHeadSize', 0.5, 'LineWidth', 2);
% Black = VB Check (Velocity from Link 4 side) - Should overlap Green
quiver(real(RB1), imag(RB1), VB_Check1x*scale, VB_Check1y*scale, 0, 'black', 'MaxHeadSize', 0.5, 'LineWidth', 1);
axis equal; grid on;

% Figure 2: Crossed Circuit
figure(2)
title('Case 2: Crossed Circuit (Velocity Vectors)'); hold on;
% Draw Linkage
plot([0 real(RA2)],[0 imag(RA2)],'g-','LineWidth',2);
plot([real(RA2) real(RB2)],[imag(RA2) imag(RB2)],'y-','LineWidth',2);
plot([real(RO4O2) real(RB2)],[imag(RO4O2) imag(RB2)],'Color',[0.5 0.5 0.5],'LineWidth',2);
plot([0 real(RO4O2)],[0 imag(RO4O2)],'m-','LineWidth',2);

% Plot Velocity Vectors
quiver(real(RA2), imag(RA2), VA_2x*scale, VA_2y*scale, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2);
quiver(real(RB2), imag(RB2), VBA_2x*scale, VBA_2y*scale, 0, 'blue', 'MaxHeadSize', 0.5, 'LineWidth', 2);
quiver(real(RB2), imag(RB2), VB_2x*scale, VB_2y*scale, 0, 'green', 'MaxHeadSize', 0.5, 'LineWidth', 2);
quiver(real(RB2), imag(RB2), VB_Check2x*scale, VB_Check2y*scale, 0, 'black', 'MaxHeadSize', 0.5, 'LineWidth', 1);
axis equal; grid on;

% Display Calculated Velocities
disp('--- Velocity Results ---');
disp(['Input w3: ', num2str(w3), ' rad/s']);
disp(' ');
disp('Case 1 (Open):');
disp(['  w2: ', num2str(w2_1), ' rad/s']);
disp(['  w4: ', num2str(w4_1), ' rad/s']);
disp(' ');
disp('Case 2 (Crossed):');
disp(['  w2: ', num2str(w2_2), ' rad/s']);
disp(['  w4: ', num2str(w4_2), ' rad/s']);
