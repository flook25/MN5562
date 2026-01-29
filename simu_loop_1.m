function y = fcn(u1,u2,u3,u4,u5,u6,u7)
% Inputs mapping:
% u1: alpha_Yellow (Input Angular Acceleration)
% u2: w_Green (Calculated Angular Velocity of Green Link)
% u3: w_Yellow (Input Angular Velocity)
% u4: w_Grey (Calculated Angular Velocity of Grey Link)
% u5: theta_Green (q2) in rad
% u6: theta_Yellow (q3) in rad
% u7: theta_Grey (q4) in rad

% ==========================================
% PARAMETERS (Loop 1: Green-Yellow-Grey)
% ==========================================
L1 = 0.210; % Ground (Pink)
L2 = 0.180; % Green (a)
L3 = 0.180; % Yellow (b)
L4 = 0.118; % Grey (c)

a = L2;
b = L3;
c = L4;
d = L1;

% ==========================================
% ASSIGN INPUTS
% ==========================================
alpha_Yellow = u1;
w_Green      = u2;
w_Yellow     = u3;
w_Grey       = u4;
q_Green      = u5;
q_Yellow     = u6;
q_Grey       = u7;

% ==========================================
% POSITION VECTORS
% ==========================================
% Green Link Vector (Link 2)
R_Green = a*exp(1i*q_Green);
R_Green_x = real(R_Green);
R_Green_y = imag(R_Green);

% Yellow Link Vector (Link 3) relative to Green
R_Yellow = b*exp(1i*q_Yellow);
R_Yellow_x = real(R_Yellow);
R_Yellow_y = imag(R_Yellow);

% Grey Link Vector (Link 4) relative to Ground (O4)
R_Grey = c*exp(1i*q_Grey);
R_Grey_x = real(R_Grey);
R_Grey_y = imag(R_Grey);

% ==========================================
% VELOCITY VECTORS (V = j*w*R)
% ==========================================
V_Green = 1i*a*w_Green*exp(1i*q_Green);
V_Green_x = real(V_Green);
V_Green_y = imag(V_Green);

V_Yellow_Rel = 1i*b*w_Yellow*exp(1i*q_Yellow);
V_Yellow_Rel_x = real(V_Yellow_Rel);
V_Yellow_Rel_y = imag(V_Yellow_Rel);

V_Grey = 1i*c*w_Grey*exp(1i*q_Grey);
V_Grey_x = real(V_Grey);
V_Grey_y = imag(V_Grey);

% ==========================================
% ACCELERATION ANALYSIS (Cramer's Rule)
% ==========================================
% Loop Equation: Green(2) + Yellow(3) - Grey(4) - Ground = 0
% Unknowns: alpha_Green, alpha_Grey
% Known Input: alpha_Yellow

A_coef = -a*sin(q_Green); 
B_coef = c*sin(q_Grey); 
D_coef = a*cos(q_Green); 
E_coef = -c*cos(q_Grey);

% RHS Terms (C and F) - Using your correct formulas
C_val = a*w_Green^2*cos(q_Green) + b*w_Yellow^2*cos(q_Yellow) + b*alpha_Yellow*sin(q_Yellow) - c*w_Grey^2*cos(q_Grey);
F_val = a*w_Green^2*sin(q_Green) + b*w_Yellow^2*sin(q_Yellow) - b*alpha_Yellow*cos(q_Yellow) - c*w_Grey^2*sin(q_Grey);

% Solve for Angular Accelerations
alpha_Green = (C_val*E_coef - B_coef*F_val) / (A_coef*E_coef - B_coef*D_coef);
alpha_Grey  = (A_coef*F_val - C_val*D_coef) / (A_coef*E_coef - B_coef*D_coef);

% ==========================================
% ACCELERATION VECTORS (A = j*alpha*R - w^2*R)
% ==========================================
A_Green_Vec = (1i*alpha_Green*a - a*w_Green^2)*exp(1i*q_Green);
A_Green_x = real(A_Green_Vec);
A_Green_y = imag(A_Green_Vec);

A_Yellow_Rel_Vec = (1i*alpha_Yellow*b - b*w_Yellow^2)*exp(1i*q_Yellow);
A_Yellow_Rel_x = real(A_Yellow_Rel_Vec);
A_Yellow_Rel_y = imag(A_Yellow_Rel_Vec);

A_Grey_Vec = (1i*alpha_Grey*c - c*w_Grey^2)*exp(1i*q_Grey);
A_Grey_x = real(A_Grey_Vec);
A_Grey_y = imag(A_Grey_Vec);

% ==========================================
% OUTPUT MAPPING
% ==========================================
xout = zeros(20,1);

% 1-2: Calculated Angular Accelerations
xout(1) = alpha_Green; 
xout(2) = alpha_Grey; 

% 3-8: Position Vector Components
xout(3) = R_Green_x;
xout(4) = R_Green_y;
xout(5) = R_Yellow_x;
xout(6) = R_Yellow_y;
xout(7) = R_Grey_x;
xout(8) = R_Grey_y;

% 9-14: Velocity Vector Components
xout(9)  = V_Green_x;
xout(10) = V_Green_y;
xout(11) = V_Yellow_Rel_x;
xout(12) = V_Yellow_Rel_y;
xout(13) = V_Grey_x;
xout(14) = V_Grey_y;

% 15-20: Acceleration Vector Components
xout(15) = A_Green_x;
xout(16) = A_Green_y;
xout(17) = A_Yellow_Rel_x;
xout(18) = A_Yellow_Rel_y;
xout(19) = A_Grey_x;
xout(20) = A_Grey_y;

y = xout;
end
