clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d) - Pink
L2 = 0.118; % Crank (a) - Cyan (Half Blue)
L3 = 0.210; % Coupler (b) - Red
L4 = 0.118; % Rocker (c) - Grey (Input Link for Inverse)

a = L2;
b = L3;
c = L4;
d = L1;

% --- 2. Input Parameter (Theta 4 Known) ---
% Ground is lifted by 0.81 degrees
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q4d = -92.2333; % Known Input Angle (Theta 4)
q4 = deg2rad(q4d) - offset; % Local Theta 4

% --- 3. Define K Constants (Inverse: Link 4 is Input) ---
% Swapping 'a' and 'c' roles from standard formula
K1 = d/c; 
K2 = d/a;
K3 = (c^2 - b^2 + a^2 + d^2)/(2*c*a); 

% Constants for Theta 3 derivation (Geometric coefficients)
% P*cos(q3) + Q*sin(q3) + R = 0
P_coeff = -2*b*(d + c*cos(q4));
Q_coeff = -2*b*c*sin(q4);
R_coeff = d^2 + c^2 + b^2 - a^2 + 2*d*c*cos(q4);

% --- 4. Calculate Coefficients for Theta 2 (Crank) ---
% Using Freudenstein Pattern A, B, C
A = cos(q4) - K1 - K2*cos(q4) + K3;
B = -2*sin(q4);
C = K1 - (K2+1)*cos(q4) + K3;

% --- 5. Calculate Coefficients for Theta 3 (Coupler) ---
% Using Geometric Pattern D, E, F derived from Loop Closure
D = R_coeff - P_coeff;
E = 2*Q_coeff;
F = R_coeff + P_coeff;

% --- 6. Solve for Theta 2 (Crank) ---
q21 = 2*atan((-B + sqrt(B^2 - 4*A*C))/(2*A)); % Solution 1
q22 = 2*atan((-B - sqrt(B^2 - 4*A*C))/(2*A)); % Solution 2

q21d = rad2deg(q21) + offset_deg;
q22d = rad2deg(q22) + offset_deg;

% --- 7. Solve for Theta 3 (Coupler) ---
q31 = 2*atan((-E + sqrt(E^2 - 4*D*F))/(2*D)); % Solution 1
q32 = 2*atan((-E - sqrt(E^2 - 4*D*F))/(2*D)); % Solution 2

q31d = rad2deg(q31) + offset_deg;
q32d = rad2deg(q32) + offset_deg;

% --- 8. Vector Calculation for Plotting ---
% Ground Vector
RO4O2 = d*exp(j*offset); 
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% Case 1: Open Circuit (Using pair 1)
RA1 = a*exp(j*(q21 + offset));       
RBA1 = b*exp(j*(q31 + offset));        
RBO4_1 = c*exp(j*(q4 + offset));    

RA1x = real(RA1); RA1y = imag(RA1);
RBA1x = real(RBA1); RBA1y = imag(RBA1); 
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);
RB1x = RA1x + RBA1x; RB1y = RA1y + RBA1y;

% Case 2: Crossed Circuit (Using pair 2)
RA2 = a*exp(j*(q22 + offset));       
RBA2 = b*exp(j*(q32 + offset));
RBO4_2 = c*exp(j*(q4 + offset));    

RA2x = real(RA2); RA2y = imag(RA2);
RBA2x = real(RBA2); RBA2y = imag(RBA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);
RB2x = RA2x + RBA2x; RB2y = RA2y + RBA2y;

% --- 9. Plotting (Position) ---
figure(1)
title('Inverse Position Analysis'); hold on;

% Plot Case 1 (Open) - Solid lines
quiver(0,0, RO4O2x, RO4O2y, 0, 'Color', [1 0 1], 'MaxHeadSize', 0.5, 'LineWidth', 4); % Ground (Pink)
quiver(0,0, RA1x, RA1y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Crank (Cyan)
quiver(RA1x, RA1y, RBA1x, RBA1y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 3); % Coupler (Red)
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 3); % Rocker (Grey)

% Plot Case 2 (Crossed) - Dashed lines for distinction
quiver(0,0, RA2x, RA2y, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 2, 'LineStyle', '--'); 
quiver(RA2x, RA2y, RBA2x, RBA2y, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2, 'LineStyle', '--');
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 2, 'LineStyle', '--');

axis equal; grid on; xlabel('x'); ylabel('y');


% --- 10. Velocity Analysis (Inverse) ---
% Assuming Link 4 (Rocker) is the Driver for this inverse calculation
w4 = 10; % Example Input Velocity rad/s (Since problem requires calculation, we assume a value)

% Inverse Velocity Formulas (Derived from w4 input)
% w2 = w4 * (c*sin(q4-q3)) / (a*sin(q2-q3))
% w3 = w4 * (c*sin(q4-q2)) / (b*sin(q3-q2))

% For Case 2 (Crossed Circuit - as per example flow)
w2_2 = (c*w4*sin(q4-q32))/(a*sin(q22-q32)); 
w3_2 = (c*w4*sin(q4-q22))/(b*sin(q32-q22));

% Velocity Vectors
VA = j*a*w2_2*exp(j*q22);
VAx = real(VA); VAy = imag(VA);

VBA = j*b*w3_2*exp(j*q32);
VBAx = real(VBA); VBAy = imag(VBA);

VB = j*c*w4*exp(j*q4);
VBx = real(VB); VBy = imag(VB);

figure(2)
title('Velocity Analysis (Crossed Circuit)'); hold on;
% Draw Linkage skeleton
plot([0 RA2x RA2x+RBA2x RO4O2x 0], [0 RA2y RA2y+RBA2y RO4O2y 0], 'k-');
% Draw Velocity Vectors
quiver(RA2x, RA2y, VAx/20, VAy/20, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
quiver(RB2x, RB2y, VBAx/20, VBAy/20, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
quiver(RB2x, RB2y, VBx/20, VBy/20, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 2); 
axis equal; grid on;


% --- 11. Acceleration Analysis (Inverse) ---
alpha4 = 0; % Example Input Accel rad/s^2

% Solve Linear Equations for alpha2 and alpha3 (Using Case 2)
% Eq: A*alpha2 + B*alpha3 = C
%     D*alpha2 + E*alpha3 = F

% Terms from acceleration loop equation (rearranged for alpha2, alpha3)
% Real part: -a*sin(q2)*alpha2 - b*sin(q3)*alpha3 = ...
% Imag part:  a*cos(q2)*alpha2 + b*cos(q3)*alpha3 = ...

A_acc = -a*sin(q22); 
B_acc = -b*sin(q32);
C_acc = a*w2_2^2*cos(q22) + b*w3_2^2*cos(q32) - c*w4^2*cos(q4) - c*alpha4*sin(q4) + d*0; % d term is 0

D_acc = a*cos(q22); 
E_acc = b*cos(q32);
F_acc = a*w2_2^2*sin(q22) + b*w3_2^2*sin(q32) - c*w4^2*sin(q4) + c*alpha4*cos(q4);

% Cramer's Rule / Determinant Solve
Det = A_acc*E_acc - B_acc*D_acc;
alpha2_2 = (C_acc*E_acc - B_acc*F_acc)/Det;
alpha3_2 = (A_acc*F_acc - C_acc*D_acc)/Det;

% Acceleration Vectors (Polar)
AA = j*a*alpha2_2*exp(j*q22) - a*w2_2^2*exp(j*q22);
AAx = real(AA); AAy = imag(AA);

ABA = j*b*alpha3_2*exp(j*q32) - b*w3_2^2*exp(j*q32);
ABAx = real(ABA); ABAy = imag(ABA);

AB = j*c*alpha4*exp(j*q4) - c*w4^2*exp(j*q4);
ABx = real(AB); ABy = imag(AB);

figure(3)
title('Acceleration Analysis (Crossed Circuit)'); hold on;
plot([0 RA2x RA2x+RBA2x RO4O2x 0], [0 RA2y RA2y+RBA2y RO4O2y 0], 'k-');
quiver(RA2x, RA2y, AAx/100, AAy/100, 0, 'cyan', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
quiver(RB2x, RB2y, ABAx/100, ABAy/100, 0, 'red', 'MaxHeadSize', 0.5, 'LineWidth', 2); 
quiver(RB2x, RB2y, ABx/100, ABy/100, 0, 'Color', [0.5 0.5 0.5], 'MaxHeadSize', 0.5, 'LineWidth', 2); 
axis equal; grid on;

% Display Results
disp('--- Position Results ---');
disp(['Input Theta 4: ', num2str(q4d)]);
disp(['Calc Theta 2 (Crossed): ', num2str(q22d)]);
disp(['Calc Theta 3 (Crossed): ', num2str(q32d)]);
