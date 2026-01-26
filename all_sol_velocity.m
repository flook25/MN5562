clear all
close all
clc

% ==========================================
% SECTION 1: PARAMETERS & POSITION ANALYSIS
% ==========================================
% --- Dimensions (Meters) ---
L1 = 0.210; % Ground
L2_Loop1 = 0.180; % Green
L3_Loop1 = 0.180; % Yellow
L4_Shared = 0.118; % Grey

L2_Loop2 = 0.118; % Cyan
L3_Loop2 = 0.210; % Red

L2_Loop3 = 0.118; % Cyan
L3_Loop3 = 0.210; % Blue
L4_Loop3 = 0.118; % Brown

d = L1;
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q3d_global = 19.94; 
q3 = deg2rad(q3d_global) - offset;

% --- LOOP 1 POSITION ---
a = L2_Loop1; b = L3_Loop1; c = L4_Shared;
K1=d/b; K2=d/c; K3=(b^2-a^2+c^2+d^2)/(2*b*c); K4=d/a; K5=(c^2-d^2-b^2-a^2)/(2*b*a);
A=cos(q3)-K1-K2*cos(q3)+K3; B=-2*sin(q3); C=K1-(K2+1)*cos(q3)+K3;
D=cos(q3)-K1+K4*cos(q3)+K5; E=-2*sin(q3); F=K1+(K4-1)*cos(q3)+K5;

q4_L1_open = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_open = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D));
q4_L1_cross = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A));
q2_L1_cross = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D));

% --- LOOP 2 POSITION ---
a = L2_Loop2; b = L3_Loop2; c = L4_Shared;
K1_L2=d/c; K2_L2=d/a; K3_L2=(c^2-b^2+a^2+d^2)/(2*c*a); K4_L2=d/b; K5_L2=(a^2-d^2-c^2-b^2)/(2*c*b);

% Case 1 (Open)
q_in_1 = q4_L1_open; 
A2_1=cos(q_in_1)-K1_L2-K2_L2*cos(q_in_1)+K3_L2; B2_1=-2*sin(q_in_1); C2_1=K1_L2-(K2_L2+1)*cos(q_in_1)+K3_L2;
D2_1=cos(q_in_1)-K1_L2+K4_L2*cos(q_in_1)+K5_L2; E2_1=-2*sin(q_in_1); F2_1=K1_L2+(K4_L2-1)*cos(q_in_1)+K5_L2;
q2_Cy_1 = 2*atan((-B2_1 + sqrt(B2_1^2 - 4*A2_1*C2_1))/(2*A2_1)); 
q3_Rd_1 = 2*atan((-E2_1 + sqrt(E2_1^2 - 4*D2_1*F2_1))/(2*D2_1));

% Case 2 (Crossed)
q_in_2 = q4_L1_cross;
A2_2=cos(q_in_2)-K1_L2-K2_L2*cos(q_in_2)+K3_L2; B2_2=-2*sin(q_in_2); C2_2=K1_L2-(K2_L2+1)*cos(q_in_2)+K3_L2;
D2_2=cos(q_in_2)-K1_L2+K4_L2*cos(q_in_2)+K5_L2; E2_2=-2*sin(q_in_2); F2_2=K1_L2+(K4_L2-1)*cos(q_in_2)+K5_L2;
q2_Cy_2 = 2*atan((-B2_2 + sqrt(B2_2^2 - 4*A2_2*C2_2))/(2*A2_2)); 
q3_Rd_2 = 2*atan((-E2_2 + sqrt(E2_2^2 - 4*D2_2*F2_2))/(2*D2_2));

% --- LOOP 3 POSITION ---
a = L2_Loop3; b = L3_Loop3; c = L4_Loop3;
K1_L3=d/a; K2_L3=d/c; K3_L3=(a^2-b^2+c^2+d^2)/(2*a*c); K4_L3=d/b; K5_L3=(c^2-d^2-a^2-b^2)/(2*a*b);

% Case 1 (Open)
q_in_3_1 = q2_Cy_1 + pi;
A3_1=cos(q_in_3_1)-K1_L3-K2_L3*cos(q_in_3_1)+K3_L3; B3_1=-2*sin(q_in_3_1); C3_1=K1_L3-(K2_L3+1)*cos(q_in_3_1)+K3_L3;
D3_1=cos(q_in_3_1)-K1_L3+K4_L3*cos(q_in_3_1)+K5_L3; E3_1=-2*sin(q_in_3_1); F3_1=K1_L3+(K4_L3-1)*cos(q_in_3_1)+K5_L3;
q4_Br_1 = 2*atan((-B3_1 + sqrt(B3_1^2 - 4*A3_1*C3_1))/(2*A3_1));
q3_Bl_1 = 2*atan((-E3_1 + sqrt(E3_1^2 - 4*D3_1*F3_1))/(2*D3_1));

% Case 2 (Crossed)
q_in_3_2 = q2_Cy_2 + pi;
A3_2=cos(q_in_3_2)-K1_L3-K2_L3*cos(q_in_3_2)+K3_L3; B3_2=-2*sin(q_in_3_2); C3_2=K1_L3-(K2_L3+1)*cos(q_in_3_2)+K3_L3;
D3_2=cos(q_in_3_2)-K1_L3+K4_L3*cos(q_in_3_2)+K5_L3; E3_2=-2*sin(q_in_3_2); F3_2=K1_L3+(K4_L3-1)*cos(q_in_3_2)+K5_L3;
q4_Br_2 = 2*atan((-B3_2 + sqrt(B3_2^2 - 4*A3_2*C3_2))/(2*A3_2));
q3_Bl_2 = 2*atan((-E3_2 + sqrt(E3_2^2 - 4*D3_2*F3_2))/(2*D3_2));

% ==========================================
% SECTION 6: VELOCITY ANALYSIS (SCALARS)
% ==========================================
w_Yellow = 25; % rad/s (Input)

% --- Loop 1 Scalars ---
a=L2_Loop1; b=L3_Loop1; c=L4_Shared;
% Case 1
w_Grey_1 = (b * w_Yellow * sin(q3 - q2_L1_open)) / (c * sin(q4_L1_open - q2_L1_open));
w_Green_1 = (b * w_Yellow * sin(q4_L1_open - q3)) / (a * sin(q4_L1_open - q2_L1_open));
% Case 2
w_Grey_2 = (b * w_Yellow * sin(q3 - q2_L1_cross)) / (c * sin(q4_L1_cross - q2_L1_cross));
w_Green_2 = (b * w_Yellow * sin(q4_L1_cross - q3)) / (a * sin(q4_L1_cross - q2_L1_cross));

% --- Loop 2 Scalars ---
a=L2_Loop2; b=L3_Loop2; c=L4_Shared;
% Case 1
w_Cyan_1 = (c * w_Grey_1 * sin(q4_L1_open - q3_Rd_1)) / (a * sin(q2_Cy_1 - q3_Rd_1));
w_Red_1 = (c * w_Grey_1 * sin(q4_L1_open - q2_Cy_1)) / (b * sin(q3_Rd_1 - q2_Cy_1));
% Case 2
w_Cyan_2 = (c * w_Grey_2 * sin(q4_L1_cross - q3_Rd_2)) / (a * sin(q2_Cy_2 - q3_Rd_2));
w_Red_2 = (c * w_Grey_2 * sin(q4_L1_cross - q2_Cy_2)) / (b * sin(q3_Rd_2 - q2_Cy_2));

% --- Loop 3 Scalars ---
a=L2_Loop3; b=L3_Loop3; c=L4_Loop3;
% Case 1
w_Brown_1 = (a * w_Cyan_1 * sin(q_in_3_1 - q3_Bl_1)) / (c * sin(q4_Br_1 - q3_Bl_1));
w_Blue_1 = (a * w_Cyan_1 * sin(q4_Br_1 - q_in_3_1)) / (b * sin(q4_Br_1 - q3_Bl_1));
% Case 2
w_Brown_2 = (a * w_Cyan_2 * sin(q_in_3_2 - q3_Bl_2)) / (c * sin(q4_Br_2 - q3_Bl_2));
w_Blue_2 = (a * w_Cyan_2 * sin(q4_Br_2 - q_in_3_2)) / (b * sin(q4_Br_2 - q3_Bl_2));

% ==========================================
% SECTION 7: VELOCITY VECTORS & COMPONENTS
% ==========================================
RO4O2 = d*exp(j*offset); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% --- Case 1 (Open) ---
% Position Origins
P_Green_1 = L2_Loop1 * exp(j*(q2_L1_open + offset));
P_Grey_1 = L4_Shared * exp(j*(q4_L1_open + offset));
P_Cyan_Down_1 = L2_Loop2 * exp(j*(q2_Cy_1 + offset));
P_Brown_1 = L4_Loop3 * exp(j*(q4_Br_1 + offset));

% Velocity Vectors (V = j*r*w*e^jtheta)
V_Green_1 = j * L2_Loop1 * w_Green_1 * exp(j*(q2_L1_open + offset));
V_Yellow_Rel_1 = j * L3_Loop1 * w_Yellow * exp(j*(q3 + offset));
V_Grey_1 = j * L4_Shared * w_Grey_1 * exp(j*(q4_L1_open + offset));
V_Cyan_Down_1 = j * L2_Loop2 * w_Cyan_1 * exp(j*(q2_Cy_1 + offset));
V_Brown_1 = j * L4_Loop3 * w_Brown_1 * exp(j*(q4_Br_1 + offset));

% Components (Real/Imag)
V_Green_1x = real(V_Green_1); V_Green_1y = imag(V_Green_1);
V_Yellow_1x = real(V_Yellow_Rel_1); V_Yellow_1y = imag(V_Yellow_Rel_1);
V_Grey_1x = real(V_Grey_1); V_Grey_1y = imag(V_Grey_1);
V_Cyan_1x = real(V_Cyan_Down_1); V_Cyan_1y = imag(V_Cyan_Down_1);
V_Brown_1x = real(V_Brown_1); V_Brown_1y = imag(V_Brown_1);

% --- Case 2 (Crossed) ---
% Position Origins
P_Green_2 = L2_Loop1 * exp(j*(q2_L1_cross + offset));
P_Grey_2 = L4_Shared * exp(j*(q4_L1_cross + offset));
P_Cyan_Down_2 = L2_Loop2 * exp(j*(q2_Cy_2 + offset));
P_Brown_2 = L4_Loop3 * exp(j*(q4_Br_2 + offset));

% Velocity Vectors
V_Green_2 = j * L2_Loop1 * w_Green_2 * exp(j*(q2_L1_cross + offset));
V_Yellow_Rel_2 = j * L3_Loop1 * w_Yellow * exp(j*(q3 + offset));
V_Grey_2 = j * L4_Shared * w_Grey_2 * exp(j*(q4_L1_cross + offset));
V_Cyan_Down_2 = j * L2_Loop2 * w_Cyan_2 * exp(j*(q2_Cy_2 + offset));
V_Brown_2 = j * L4_Loop3 * w_Brown_2 * exp(j*(q4_Br_2 + offset));

% Components
V_Green_2x = real(V_Green_2); V_Green_2y = imag(V_Green_2);
V_Yellow_2x = real(V_Yellow_Rel_2); V_Yellow_2y = imag(V_Yellow_Rel_2);
V_Grey_2x = real(V_Grey_2); V_Grey_2y = imag(V_Grey_2);
V_Cyan_2x = real(V_Cyan_Down_2); V_Cyan_2y = imag(V_Cyan_Down_2);
V_Brown_2x = real(V_Brown_2); V_Brown_2y = imag(V_Brown_2);

% ==========================================
% PLOTTING (With Scaling)
% ==========================================
Scale = 1/50;

% --- Figure 1: Case 1 ---
figure(1)
hold on;
% Mechanism
plot([0 real(P_Green_1)], [0 imag(P_Green_1)], 'g', 'LineWidth', 2);
plot([real(P_Green_1) real(P_Grey_1)+RO4O2x], [imag(P_Green_1) imag(P_Grey_1)+RO4O2y], 'y', 'LineWidth', 2);
plot([RO4O2x real(P_Grey_1)+RO4O2x], [RO4O2y imag(P_Grey_1)+RO4O2y], 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
plot([RO4O2x real(P_Cyan_Down_1)+RO4O2x], [RO4O2y imag(P_Cyan_Down_1)+RO4O2y], 'c', 'LineWidth', 2);
plot([RO4O2x real(P_Brown_1)+RO4O2x], [RO4O2y imag(P_Brown_1)+RO4O2y], 'Color', [0.85 0.5 0.1], 'LineWidth', 2);

% Velocities (Using Components)
quiver(real(P_Green_1), imag(P_Green_1), V_Green_1x*Scale, V_Green_1y*Scale, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(P_Green_1), imag(P_Green_1), V_Yellow_1x*Scale, V_Yellow_1y*Scale, 0, 'y', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(P_Grey_1)+RO4O2x, imag(P_Grey_1)+RO4O2y, V_Grey_1x*Scale, V_Grey_1y*Scale, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(P_Cyan_Down_1)+RO4O2x, imag(P_Cyan_Down_1)+RO4O2y, V_Cyan_1x*Scale, V_Cyan_1y*Scale, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(P_Brown_1)+RO4O2x, imag(P_Brown_1)+RO4O2y, V_Brown_1x*Scale, V_Brown_1y*Scale, 0, 'Color', [0.85 0.5 0.1], 'LineWidth', 2, 'MaxHeadSize', 0.5);
axis equal; grid on; title('Case 1: Velocity');

% --- Figure 2: Case 2 ---
figure(2)
hold on;
% Mechanism
plot([0 real(P_Green_2)], [0 imag(P_Green_2)], 'g', 'LineWidth', 2);
plot([real(P_Green_2) real(P_Grey_2)+RO4O2x], [imag(P_Green_2) imag(P_Grey_2)+RO4O2y], 'y', 'LineWidth', 2);
plot([RO4O2x real(P_Grey_2)+RO4O2x], [RO4O2y imag(P_Grey_2)+RO4O2y], 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
plot([RO4O2x real(P_Cyan_Down_2)+RO4O2x], [RO4O2y imag(P_Cyan_Down_2)+RO4O2y], 'c', 'LineWidth', 2);
plot([RO4O2x real(P_Brown_2)+RO4O2x], [RO4O2y imag(P_Brown_2)+RO4O2y], 'Color', [0.85 0.5 0.1], 'LineWidth', 2);

% Velocities
quiver(real(P_Green_2), imag(P_Green_2), V_Green_2x*Scale, V_Green_2y*Scale, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(P_Green_2), imag(P_Green_2), V_Yellow_2x*Scale, V_Yellow_2y*Scale, 0, 'y', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(P_Grey_2)+RO4O2x, imag(P_Grey_2)+RO4O2y, V_Grey_2x*Scale, V_Grey_2y*Scale, 0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(P_Cyan_Down_2)+RO4O2x, imag(P_Cyan_Down_2)+RO4O2y, V_Cyan_2x*Scale, V_Cyan_2y*Scale, 0, 'c', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(real(P_Brown_2)+RO4O2x, imag(P_Brown_2)+RO4O2y, V_Brown_2x*Scale, V_Brown_2y*Scale, 0, 'Color', [0.85 0.5 0.1], 'LineWidth', 2, 'MaxHeadSize', 0.5);
axis equal; grid on; title('Case 2: Velocity');

% --- Display Results ---
disp('--- VELOCITY RESULTS (rad/s) ---');
disp('CASE 1 (OPEN):');
disp(['  w_Yellow (Input): ', num2str(w_Yellow)]);
disp(['  w_Green: ', num2str(w_Green_1)]);
disp(['  w_Grey:  ', num2str(w_Grey_1)]);
disp(['  w_Cyan:  ', num2str(w_Cyan_1)]);
disp(['  w_Red:   ', num2str(w_Red_1)]);
disp(['  w_Blue:  ', num2str(w_Blue_1)]);
disp(['  w_Brown: ', num2str(w_Brown_1)]);
