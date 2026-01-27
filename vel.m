clear all
close all
clc

% ==========================================
% SECTION 1: PARAMETERS & INPUT
% ==========================================
L1 = 0.210; % Ground (Pink)
L2_Loop1 = 0.180; % Green
L3_Loop1 = 0.180; % Yellow
L4_Shared = 0.118; % Grey

L2_Loop2 = 0.118; % Cyan
L3_Loop2 = 0.210; % Red

L2_Loop3 = 0.118; % Cyan (Extended)
L3_Loop3 = 0.210; % Blue
L4_Loop3 = 0.118; % Brown

d = L1;
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q3d_global = 19.94; 
q3 = deg2rad(q3d_global) - offset;

w_Yellow = -2.2; % Input Angular Velocity

% ==========================================
% SECTION 2: POSITION ANALYSIS (CALCULATE ANGLES)
% ==========================================

% --- LOOP 1: Green-Yellow-Grey ---
a = L2_Loop1; b = L3_Loop1; c = L4_Shared;
K1=d/b; K2=d/c; K3=(b^2-a^2+c^2+d^2)/(2*b*c); K4=d/a; K5=(c^2-d^2-b^2-a^2)/(2*b*a);
A=cos(q3)-K1-K2*cos(q3)+K3; B=-2*sin(q3); C=K1-(K2+1)*cos(q3)+K3;
D=cos(q3)-K1+K4*cos(q3)+K5; E=-2*sin(q3); F=K1+(K4-1)*cos(q3)+K5;

q4_L1_open = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_open = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));
q4_L1_cross = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_cross = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D));

% --- LOOP 2: Cyan-Red-Grey ---
a = L2_Loop2; b = L3_Loop2; c = L4_Shared;
K1_L2=d/c; K2_L2=d/a; K3_L2=(c^2-b^2+a^2+d^2)/(2*c*a); K4_L2=d/b; K5_L2=(a^2-d^2-c^2-b^2)/(2*c*b);

% Case 1 (Open Input)
q_in_1 = q4_L1_open;
A2_1=cos(q_in_1)-K1_L2-K2_L2*cos(q_in_1)+K3_L2; B2_1=-2*sin(q_in_1); C2_1=K1_L2-(K2_L2+1)*cos(q_in_1)+K3_L2;
D2_1=cos(q_in_1)-K1_L2+K4_L2*cos(q_in_1)+K5_L2; E2_1=-2*sin(q_in_1); F2_1=K1_L2+(K4_L2-1)*cos(q_in_1)+K5_L2;
q2_Cyan_1 = 2*atan((-B2_1 + sqrt(B2_1^2 - 4*A2_1*C2_1))/(2*A2_1)); 
q3_Red_1 = 2*atan((-E2_1 + sqrt(E2_1^2 - 4*D2_1*F2_1))/(2*D2_1));

% Case 2 (Crossed Input)
q_in_2 = q4_L1_cross;
A2_2=cos(q_in_2)-K1_L2-K2_L2*cos(q_in_2)+K3_L2; B2_2=-2*sin(q_in_2); C2_2=K1_L2-(K2_L2+1)*cos(q_in_2)+K3_L2;
D2_2=cos(q_in_2)-K1_L2+K4_L2*cos(q_in_2)+K5_L2; E2_2=-2*sin(q_in_2); F2_2=K1_L2+(K4_L2-1)*cos(q_in_2)+K5_L2;
q2_Cyan_2 = 2*atan((-B2_2 + sqrt(B2_2^2 - 4*A2_2*C2_2))/(2*A2_2)); 
q3_Red_2 = 2*atan((-E2_2 + sqrt(E2_2^2 - 4*D2_2*F2_2))/(2*D2_2));

% --- LOOP 3: Cyan-Blue-Brown ---
a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;
K1_L3=d/a; K2_L3=d/c; K3_L3=(a^2-b^2+c^2+d^2)/(2*a*c); K4_L3=d/b; K5_L3=(c^2-d^2-a^2-b^2)/(2*a*b);

% Case 1 (Open Input)
q_in_3_1 = q2_Cyan_1 + pi;
A3_1=cos(q_in_3_1)-K1_L3-K2_L3*cos(q_in_3_1)+K3_L3; B3_1=-2*sin(q_in_3_1); C3_1=K1_L3-(K2_L3+1)*cos(q_in_3_1)+K3_L3;
D3_1=cos(q_in_3_1)-K1_L3+K4_L3*cos(q_in_3_1)+K5_L3; E3_1=-2*sin(q_in_3_1); F3_1=K1_L3+(K4_L3-1)*cos(q_in_3_1)+K5_L3;
q4_Brown_1 = 2*atan((-B3_1 + sqrt(B3_1^2 - 4*A3_1*C3_1))/(2*A3_1));
q3_Blue_1 = 2*atan((-E3_1 + sqrt(E3_1^2 - 4*D3_1*F3_1))/(2*D3_1));

% Case 2 (Crossed Input)
q_in_3_2 = q2_Cyan_2 + pi;
A3_2=cos(q_in_3_2)-K1_L3-K2_L3*cos(q_in_3_2)+K3_L3; B3_2=-2*sin(q_in_3_2); C3_2=K1_L3-(K2_L3+1)*cos(q_in_3_2)+K3_L3;
D3_2=cos(q_in_3_2)-K1_L3+K4_L3*cos(q_in_3_2)+K5_L3; E3_2=-2*sin(q_in_3_2); F3_2=K1_L3+(K4_L3-1)*cos(q_in_3_2)+K5_L3;
q4_Brown_2 = 2*atan((-B3_2 + sqrt(B3_2^2 - 4*A3_2*C3_2))/(2*A3_2));
q3_Blue_2 = 2*atan((-E3_2 + sqrt(E3_2^2 - 4*D3_2*F3_2))/(2*D3_2));

% --- DISPLAY POSITION RESULTS ---
disp('=== POSITION RESULTS (Degrees) ===');
disp('CASE 1 (OPEN):');
disp(['  Green:  ', num2str(rad2deg(q2_L1_open)+offset_deg)]);
disp(['  Yellow: ', num2str(q3d_global)]);
disp(['  Grey:   ', num2str(rad2deg(q4_L1_open)+offset_deg)]);
disp(['  Cyan:   ', num2str(rad2deg(q2_Cyan_1)+offset_deg)]);
disp(['  Red:    ', num2str(rad2deg(q3_Red_1)+offset_deg)]);
disp(['  Blue:   ', num2str(rad2deg(q3_Blue_1)+offset_deg)]);
disp(['  Brown:  ', num2str(rad2deg(q4_Brown_1)+offset_deg)]);
disp(' ');
disp('CASE 2 (CROSSED):');
disp(['  Green:  ', num2str(rad2deg(q2_L1_cross)+offset_deg)]);
disp(['  Yellow: ', num2str(q3d_global)]); % <--- ADDED YELLOW
disp(['  Grey:   ', num2str(rad2deg(q4_L1_cross)+offset_deg)]);
disp(['  Cyan:   ', num2str(rad2deg(q2_Cyan_2)+offset_deg)]);
disp(['  Red:    ', num2str(rad2deg(q3_Red_2)+offset_deg)]);
disp(['  Blue:   ', num2str(rad2deg(q3_Blue_2)+offset_deg)]);
disp(['  Brown:  ', num2str(rad2deg(q4_Brown_2)+offset_deg)]);
disp(' ');

% ==========================================
% SECTION 3: VELOCITY ANALYSIS (CALCULATE OMEGAS)
% ==========================================
% w_Yellow = -2.2

% --- Loop 1 ---
a=L2_Loop1; b=L3_Loop1; c=L4_Shared;
% Case 1
w_Grey_1 = (b * w_Yellow * sin(q3 - q2_L1_open)) / (c * sin(q4_L1_open - q2_L1_open));
w_Green_1 = (b * w_Yellow * sin(q4_L1_open - q3)) / (a * sin(q4_L1_open - q2_L1_open));
% Case 2
w_Grey_2 = (b * w_Yellow * sin(q3 - q2_L1_cross)) / (c * sin(q4_L1_cross - q2_L1_cross));
w_Green_2 = (b * w_Yellow * sin(q4_L1_cross - q3)) / (a * sin(q4_L1_cross - q2_L1_cross));

% --- Loop 2 ---
a=L2_Loop2; b=L3_Loop2; c=L4_Shared;
% Case 1
w_Cyan_1 = (c * w_Grey_1 * sin(q4_L1_open - q3_Red_1)) / (a * sin(q2_Cyan_1 - q3_Red_1));
w_Red_1 = (c * w_Grey_1 * sin(q4_L1_open - q2_Cyan_1)) / (b * sin(q3_Red_1 - q2_Cyan_1));
% Case 2
w_Cyan_2 = (c * w_Grey_2 * sin(q4_L1_cross - q3_Red_2)) / (a * sin(q2_Cyan_2 - q3_Red_2));
w_Red_2 = (c * w_Grey_2 * sin(q4_L1_cross - q2_Cyan_2)) / (b * sin(q3_Red_2 - q2_Cyan_2));

% --- Loop 3 ---
a=L2_Loop3; b=L3_Loop3; c=L4_Loop3;
% Case 1
w_Brown_1 = (a * w_Cyan_1 * sin(q_in_3_1 - q3_Blue_1)) / (c * sin(q4_Brown_1 - q3_Blue_1));
w_Blue_1 = (a * w_Cyan_1 * sin(q4_Brown_1 - q_in_3_1)) / (b * sin(q4_Brown_1 - q3_Blue_1));
% Case 2
w_Brown_2 = (a * w_Cyan_2 * sin(q_in_3_2 - q3_Blue_2)) / (c * sin(q4_Brown_2 - q3_Blue_2));
w_Blue_2 = (a * w_Cyan_2 * sin(q4_Brown_2 - q_in_3_2)) / (b * sin(q4_Brown_2 - q3_Blue_2));

% --- DISPLAY VELOCITIES ---
disp('=== VELOCITY RESULTS (rad/s) ===');
disp('CASE 1 (OPEN):');
disp(['  w_Green: ', num2str(w_Green_1)]);
disp(['  w_Yellow (Input): ', num2str(w_Yellow)]);
disp(['  w_Grey:  ', num2str(w_Grey_1)]);
disp(['  w_Cyan:  ', num2str(w_Cyan_1)]);
disp(['  w_Red:   ', num2str(w_Red_1)]);
disp(['  w_Blue:  ', num2str(w_Blue_1)]);
disp(['  w_Brown: ', num2str(w_Brown_1)]);
disp(' ');
disp('CASE 2 (CROSSED):');
disp(['  w_Green: ', num2str(w_Green_2)]);
disp(['  w_Yellow (Input): ', num2str(w_Yellow)]); % <--- ADDED YELLOW
disp(['  w_Grey:  ', num2str(w_Grey_2)]);
disp(['  w_Cyan:  ', num2str(w_Cyan_2)]);
disp(['  w_Red:   ', num2str(w_Red_2)]);
disp(['  w_Blue:  ', num2str(w_Blue_2)]);
disp(['  w_Brown: ', num2str(w_Brown_2)]);

% ==========================================
% SECTION 4: VECTORS & PLOTTING
% ==========================================
Scale = 0.6;
RO4O2 = L1*exp(1i*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- CASE 1 (OPEN) ---
figure(1)
hold on; title('Case 1: Open Circuit (Position & Velocity)');

% 1. Create Vectors
R_Green = L2_Loop1*exp(1i*(q2_L1_open+offset));
R_Yellow = L3_Loop1*exp(1i*(q3+offset));
R_Grey = L4_Shared*exp(1i*(q4_L1_open+offset));
R_Cyan = L2_Loop2*exp(1i*(q2_Cyan_1+offset));
R_Red = L3_Loop2*exp(1i*(q3_Red_1+offset));
R_Cyan_Up = L2_Loop3*exp(1i*(q_in_3_1+offset));
R_Blue = L3_Loop3*exp(1i*(q3_Blue_1+offset));
R_Brown = L4_Loop3*exp(1i*(q4_Brown_1+offset));

% 2. Plot Position
plot([0 real(R_Green)], [0 imag(R_Green)], 'g-', 'LineWidth', 2);
plot([real(R_Green) real(R_Grey)+RO4O2x], [imag(R_Green) imag(R_Grey)+RO4O2y], 'y-', 'LineWidth', 2);
plot([RO4O2x real(R_Grey)+RO4O2x], [RO4O2y imag(R_Grey)+RO4O2y], 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
plot([0 RO4O2x], [0 RO4O2y], 'm-', 'LineWidth', 2);
plot([RO4O2x real(R_Cyan)+RO4O2x], [RO4O2y imag(R_Cyan)+RO4O2y], 'c-', 'LineWidth', 2);
plot([real(R_Cyan)+RO4O2x real(R_Grey)+RO4O2x], [imag(R_Cyan)+RO4O2y imag(R_Grey)+RO4O2y], 'r-', 'LineWidth', 2);
plot([RO4O2x real(R_Cyan_Up)+RO4O2x], [RO4O2y imag(R_Cyan_Up)+RO4O2y], 'c:', 'LineWidth', 2);
plot([real(R_Cyan_Up)+RO4O2x real(R_Brown)+RO4O2x], [imag(R_Cyan_Up)+RO4O2y imag(R_Brown)+RO4O2y], 'b-', 'LineWidth', 2);
plot([RO4O2x real(R_Brown)+RO4O2x], [RO4O2y imag(R_Brown)+RO4O2y], 'Color', [0.6 0.3 0], 'LineWidth', 2);

% 3. Calculate Velocities
V_Green = 1i * R_Green * w_Green_1;
V_Yellow_Rel = 1i * R_Yellow * w_Yellow;
V_Grey = 1i * R_Grey * w_Grey_1;
V_Cyan = 1i * R_Cyan * w_Cyan_1;
V_Cyan_Up = 1i * R_Cyan_Up * w_Cyan_1;
V_Brown = 1i * R_Brown * w_Brown_1;

% 4. Plot Velocities
quiver(real(R_Green), imag(R_Green), real(V_Green)*Scale, imag(V_Green)*Scale, 0, 'Color', 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Green), imag(R_Green), real(V_Yellow_Rel)*Scale, imag(V_Yellow_Rel)*Scale, 0, 'Color', 'y', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Grey)+RO4O2x, imag(R_Grey)+RO4O2y, real(V_Grey)*Scale, imag(V_Grey)*Scale, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan)+RO4O2x, imag(R_Cyan)+RO4O2y, real(V_Cyan)*Scale, imag(V_Cyan)*Scale, 0, 'c', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_Up)+RO4O2x, imag(R_Cyan_Up)+RO4O2y, real(V_Cyan_Up)*Scale, imag(V_Cyan_Up)*Scale, 0, 'c', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Brown)+RO4O2x, imag(R_Brown)+RO4O2y, real(V_Brown)*Scale, imag(V_Brown)*Scale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

axis equal; grid on;

% --- CASE 2 (CROSSED) ---
figure(2)
hold on; title('Case 2: Crossed Circuit (Position & Velocity)');

% 1. Create Vectors
R_Green_2 = L2_Loop1*exp(1i*(q2_L1_cross+offset));
R_Yellow_2 = L3_Loop1*exp(1i*(q3+offset));
R_Grey_2 = L4_Shared*exp(1i*(q4_L1_cross+offset));
R_Cyan_2 = L2_Loop2*exp(1i*(q2_Cyan_2+offset));
R_Red_2 = L3_Loop2*exp(1i*(q3_Red_2+offset));
R_Cyan_Up_2 = L2_Loop3*exp(1i*(q_in_3_2+offset));
R_Blue_2 = L3_Loop3*exp(1i*(q3_Blue_2+offset));
R_Brown_2 = L4_Loop3*exp(1i*(q4_Brown_2+offset));

% 2. Plot Position
plot([0 RO4O2x], [0 RO4O2y], 'm-', 'LineWidth', 2);
plot([0 real(R_Green_2)], [0 imag(R_Green_2)], 'g-', 'LineWidth', 2);
plot([real(R_Green_2) real(R_Grey_2)+RO4O2x], [imag(R_Green_2) imag(R_Grey_2)+RO4O2y], 'y-', 'LineWidth', 2);
plot([RO4O2x real(R_Grey_2)+RO4O2x], [RO4O2y imag(R_Grey_2)+RO4O2y], 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
plot([RO4O2x real(R_Cyan_2)+RO4O2x], [RO4O2y imag(R_Cyan_2)+RO4O2y], 'c-', 'LineWidth', 2);
plot([real(R_Cyan_2)+RO4O2x real(R_Grey_2)+RO4O2x], [imag(R_Cyan_2)+RO4O2y imag(R_Grey_2)+RO4O2y], 'r-', 'LineWidth', 2);
plot([RO4O2x real(R_Cyan_Up_2)+RO4O2x], [RO4O2y imag(R_Cyan_Up_2)+RO4O2y], 'c:', 'LineWidth', 2);
plot([real(R_Cyan_Up_2)+RO4O2x real(R_Brown_2)+RO4O2x], [imag(R_Cyan_Up_2)+RO4O2y imag(R_Brown_2)+RO4O2y], 'b-', 'LineWidth', 2);
plot([RO4O2x real(R_Brown_2)+RO4O2x], [RO4O2y imag(R_Brown_2)+RO4O2y], 'Color', [0.6 0.3 0], 'LineWidth', 2);

% 3. Calculate Velocities
V_Green_2 = 1i * R_Green_2 * w_Green_2;
V_Yellow_Rel_2 = 1i * R_Yellow_2 * w_Yellow;
V_Grey_2 = 1i * R_Grey_2 * w_Grey_2;
V_Cyan_2 = 1i * R_Cyan_2 * w_Cyan_2;
V_Cyan_Up_2 = 1i * R_Cyan_Up_2 * w_Cyan_2;
V_Brown_2 = 1i * R_Brown_2 * w_Brown_2;

% 4. Plot Velocities
quiver(real(R_Green_2), imag(R_Green_2), real(V_Green_2)*Scale, imag(V_Green_2)*Scale, 0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Green_2), imag(R_Green_2), real(V_Yellow_Rel_2)*Scale, imag(V_Yellow_Rel_2)*Scale, 0, 'y', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Grey_2)+RO4O2x, imag(R_Grey_2)+RO4O2y, real(V_Grey_2)*Scale, imag(V_Grey_2)*Scale, 0, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_2)+RO4O2x, imag(R_Cyan_2)+RO4O2y, real(V_Cyan_2)*Scale, imag(V_Cyan_2)*Scale, 0, 'c', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Cyan_Up_2)+RO4O2x, imag(R_Cyan_Up_2)+RO4O2y, real(V_Cyan_Up_2)*Scale, imag(V_Cyan_Up_2)*Scale, 0, 'c', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
quiver(real(R_Brown_2)+RO4O2x, imag(R_Brown_2)+RO4O2y, real(V_Brown_2)*Scale, imag(V_Brown_2)*Scale, 0, 'Color', [0.6 0.3 0], 'LineWidth', 1.5, 'MaxHeadSize', 0.5);

axis equal; grid on;
