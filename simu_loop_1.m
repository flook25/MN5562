function y = fcn(u1,u2,u3,u4,u5,u6,u7)
% Inputs mapping (Assuming inputs from Simulink are ordered by Link 2, 3, 4):
% u1: alpha_Yellow (Input Angular Acceleration)
% u2: w_Green (Link 2)
% u3: w_Yellow (Link 3 - Input)
% u4: w_Grey (Link 4)
% u5: q_Green (Link 2) - rad
% u6: q_Yellow (Link 3) - rad
% u7: q_Grey (Link 4) - rad

% ==========================================
% PARAMETERS (Loop 1: Green-Yellow-Grey)
% ==========================================
L1 = 0.210; % Ground (d)
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
% Note: Yellow (Link 3) is INPUT, Green (2) and Grey (4) are UNKNOWNS
alpha_Yellow = u1; 

w_Green      = u2;
w_Yellow     = u3;
w_Grey       = u4;

q_Green      = u5;
q_Yellow     = u6;
q_Grey       = u7;

% ==========================================
% POSITION VECTORS (R = L*exp(j*theta))
% ==========================================
% RA -> Green Link (Link 2)
R_Green = a*exp(1i*q_Green);
RAx = real(R_Green);
RAy = imag(R_Green);

% RBA -> Yellow Link (Link 3)
R_Yellow = b*exp(1i*q_Yellow);
RBAx = real(R_Yellow);
RBAy = imag(R_Yellow);

% RB -> Grey Link (Link 4) - Relative to Ground O4
R_Grey = c*exp(1i*q_Grey);
RBx = real(R_Grey);
RBy = imag(R_Grey);

% ==========================================
% VELOCITY VECTORS (V = j*w*R)
% ==========================================
% VA -> Velocity of Green Tip
V_Green = 1i*a*w_Green*exp(1i*q_Green);
VAx = real(V_Green);
VAy = imag(V_Green);

% VBA -> Relative Velocity of Yellow
V_Yellow_Rel = 1i*b*w_Yellow*exp(1i*q_Yellow);
VBAx = real(V_Yellow_Rel);
VBAy = imag(V_Yellow_Rel);

% VB -> Velocity of Grey Tip
V_Grey = 1i*c*w_Grey*exp(1i*q_Grey);
VBx = real(V_Grey);
VBy = imag(V_Grey);

% ==========================================
% ACCELERATION ANALYSIS (Analytical Method)
% ==========================================
% Solving for alpha_Green and alpha_Grey using Cramer's Rule
% Equation: A*x + B*y = C, D*x + E*y = F
% Where x = alpha_Green, y = alpha_Grey

A_coef = -a*sin(q_Green); 
B_coef = c*sin(q_Grey); 
D_coef = a*cos(q_Green); 
E_coef = -c*cos(q_Grey);

% RHS Terms (Knowns: w terms and alpha_Yellow)
C_val = a*w_Green^2*cos(q_Green) + b*w_Yellow^2*cos(q_Yellow) + b*alpha_Yellow*sin(q_Yellow) - c*w_Grey^2*cos(q_Grey);
F_val = a*w_Green^2*sin(q_Green) + b*w_Yellow^2*sin(q_Yellow) - b*alpha_Yellow*cos(q_Yellow) - c*w_Grey^2*sin(q_Grey);

% Solve Unknown Alphas
alpha_Green = (C_val*E_coef - B_coef*F_val) / (A_coef*E_coef - B_coef*D_coef);
alpha_Grey  = (A_coef*F_val - C_val*D_coef) / (A_coef*E_coef - B_coef*D_coef);

% ==========================================
% ACCELERATION VECTORS (A = j*alpha*R - w^2*R)
% ==========================================
% AA -> Acceleration of Green
A_Green_Vec = (1i*alpha_Green*a - a*w_Green^2)*exp(1i*q_Green);
AAx = real(A_Green_Vec);
AAy = imag(A_Green_Vec);

% ABA -> Acceleration of Yellow (Relative)
A_Yellow_Vec = (1i*alpha_Yellow*b - b*w_Yellow^2)*exp(1i*q_Yellow);
ABAx = real(A_Yellow_Vec);
ABAy = imag(A_Yellow_Vec);

% AB -> Acceleration of Grey
A_Grey_Vec = (1i*alpha_Grey*c - c*w_Grey^2)*exp(1i*q_Grey);
ABx = real(A_Grey_Vec);
ABy = imag(A_Grey_Vec);

% ==========================================
% OUTPUT MAPPING (Order fixed as per request)
% ==========================================
xout = zeros(20,1);

% 1-2: Angular Accelerations (Outputs)
xout(1) = alpha_Green;  % alpha32 in your example template (Unknown 1)
xout(2) = alpha_Grey;   % alpha42 in your example template (Unknown 2)

% 3-8: Position Vectors (Components)
xout(3) = RAx;   % Green X
xout(4) = RAy;   % Green Y
xout(5) = RBAx;  % Yellow X
xout(6) = RBAy;  % Yellow Y
xout(7) = RBx;   % Grey X
xout(8) = RBy;   % Grey Y

% 9-14: Velocity Vectors (Components)
xout(9)  = VAx;
xout(10) = VAy;
xout(11) = VBAx;
xout(12) = VBAy;
xout(13) = VBx;
xout(14) = VBy;

% 15-20: Acceleration Vectors (Components)
xout(15) = AAx;
xout(16) = AAy;
xout(17) = ABAx;
xout(18) = ABAy;
xout(19) = ABx;
xout(20) = ABy;

y = xout;
end
