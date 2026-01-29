function y = fcn(u1,u2,u3,u4,u5,u6,u7)
% Inputs mapping:
% u1: alpha_Yellow (Input Angular Acceleration)
% u2: w_Green (Link 2)
% u3: w_Yellow (Link 3 - Input)
% u4: w_Grey (Link 4)
% u5: q_Green (Link 2) - rad
% u6: q_Yellow (Link 3) - rad
% u7: q_Grey (Link 4) - rad

% ==========================================
% PARAMETERS
% ==========================================
L1 = 0.210; % Ground
L2 = 0.180; % Green
L3 = 0.180; % Yellow
L4 = 0.118; % Grey

a = L2;
b = L3;
c = L4;
d = L1;

% ==========================================
% ASSIGN INPUTS
% ==========================================
alpha_Yellow = u1; % Known Input

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
RA = a*exp(1i*q_Green);
RAx = real(RA);
RAy = imag(RA);

% RBA -> Yellow Link (Link 3)
RBA = b*exp(1i*q_Yellow);
RBAx = real(RBA);
RBAy = imag(RBA);

% RB -> Grey Link (Link 4)
RB = c*exp(1i*q_Grey);
RBx = real(RB);
RBy = imag(RB);

% ==========================================
% VELOCITY VECTORS (V = j*w*R)
% ==========================================
% VA -> Green Velocity
VA = 1i*a*w_Green*exp(1i*q_Green);
VAx = real(VA);
VAy = imag(VA);

% VBA -> Yellow Velocity
VBA = 1i*b*w_Yellow*exp(1i*q_Yellow);
VBAx = real(VBA);
VBAy = imag(VBA);

% VB -> Grey Velocity
VB = 1i*c*w_Grey*exp(1i*q_Grey);
VBx = real(VB);
VBy = imag(VB);

% ==========================================
% ACCELERATION ANALYSIS (Analytical Method)
% ==========================================
% Solving for alpha_Green and alpha_Grey
% Loop Equation: Green + Yellow - Grey = 0
% A*alpha_Green + B*alpha_Grey = C
% D*alpha_Green + E*alpha_Grey = F

A_coef = -a*sin(q_Green); 
B_coef = c*sin(q_Grey); 
D_coef = a*cos(q_Green); 
E_coef = -c*cos(q_Grey);

% RHS Terms (C and F contain knowns: w^2 and alpha_Yellow)
C_val = a*w_Green^2*cos(q_Green) + b*w_Yellow^2*cos(q_Yellow) + b*alpha_Yellow*sin(q_Yellow) - c*w_Grey^2*cos(q_Grey);
F_val = a*w_Green^2*sin(q_Green) + b*w_Yellow^2*sin(q_Yellow) - b*alpha_Yellow*cos(q_Yellow) - c*w_Grey^2*sin(q_Grey);

% Solve Unknowns (Cramer's Rule format)
alpha_Green = (C_val*E_coef - B_coef*F_val) / (A_coef*E_coef - B_coef*D_coef);
alpha_Grey  = (A_coef*F_val - C_val*D_coef) / (A_coef*E_coef - B_coef*D_coef);

% ==========================================
% ACCELERATION VECTORS (A = j*alpha*R - w^2*R)
% ==========================================
% AA -> Acceleration of Green
AA = (1i*alpha_Green*a - a*w_Green^2)*exp(1i*q_Green);
AAx = real(AA); 
AAy = imag(AA);

% ABA -> Acceleration of Yellow
ABA = (1i*alpha_Yellow*b - b*w_Yellow^2)*exp(1i*q_Yellow);
ABAx = real(ABA); 
ABAy = imag(ABA);

% AB -> Acceleration of Grey
AB = (1i*alpha_Grey*c - c*w_Grey^2)*exp(1i*q_Grey);
ABx = real(AB); 
ABy = imag(AB);

% ==========================================
% OUTPUT MAPPING
% ==========================================
xout = zeros(20,1);
xout(1) = alpha_Green; % Unknown 1
xout(2) = alpha_Grey;  % Unknown 2
xout(3) = RAx; 
xout(4) = RAy; 
xout(5) = RBAx; 
xout(6) = RBAy; 
xout(7) = RBx; 
xout(8) = RBy; 
xout(9) = VAx; 
xout(10) = VAy; 
xout(11) = VBAx;
xout(12) = VBAy;
xout(13) = VBx;
xout(14) = VBy;
xout(15) = AAx;
xout(16) = AAy;
xout(17) = ABAx;
xout(18) = ABAy;
xout(19) = ABx;
xout(20) = ABy;

y = xout;
end
