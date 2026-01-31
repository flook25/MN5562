function y = fcn(u1, u2, u3)
alpha_Yellow = u1(1); 
w_Yellow = u2(1); 
q3_global_rad = u3(1);

% ==========================================
% SECTION 1: PARAMETERS 
% ==========================================
L1 = 0.210; d = L1;
L2_Loop1 = 0.180; L3_Loop1 = 0.180; L4_Shared = 0.118;
L2_Loop2 = 0.118; L3_Loop2 = 0.210;
L2_Loop3 = 0.118; L3_Loop3 = 0.210; L4_Loop3 = 0.118;

offset = deg2rad(0.81);
q3 = q3_global_rad - offset;

% ==========================================
% SECTION 2: POSITION ANALYSIS (CROSSED CIRCUIT)
% ==========================================
% --- LOOP 1 ---
a = L2_Loop1; b = L3_Loop1; c = L4_Shared;
K1=d/b; K2=d/c; K3=(b^2-a^2+c^2+d^2)/(2*b*c); K4=d/a; K5=(c^2-d^2-b^2-a^2)/(2*b*a);
A1=cos(q3)-K1-K2*cos(q3)+K3; B1=-2*sin(q3); C1=K1-(K2+1)*cos(q3)+K3;
D1=cos(q3)-K1+K4*cos(q3)+K5; E1=-2*sin(q3); F1=K1+(K4-1)*cos(q3)+K5;

q4_L1 = 2*atan(real((-B1 + sqrt(complex(B1^2 - 4*A1*C1)))/(2*A1)));
q2_L1 = 2*atan(real((-E1 + sqrt(complex(E1^2 - 4*D1*F1)))/(2*D1)));

% --- LOOP 2 ---
a2 = L2_Loop2; b2 = L3_Loop2; c2 = L4_Shared;
K1_L2=d/c2; K2_L2=d/a2; K3_L2=(c2^2-b2^2+a2^2+d^2)/(2*c2*a2); K4_L2=d/b2; K5_L2=(a2^2-d^2-c2^2-b2^2)/(2*c2*b2);
q_in_2 = q4_L1;
A2=cos(q_in_2)-K1_L2-K2_L2*cos(q_in_2)+K3_L2; B2=-2*sin(q_in_2); C2=K1_L2-(K2_L2+1)*cos(q_in_2)+K3_L2;
D2=cos(q_in_2)-K1_L2+K4_L2*cos(q_in_2)+K5_L2; E2=-2*sin(q_in_2); F2=K1_L2+(K4_L2-1)*cos(q_in_2)+K5_L2;
q2_Cyan = 2*atan(real((-B2 + sqrt(complex(B2^2 - 4*A2*C2)))/(2*A2)));
q3_Red = 2*atan(real((-E2 + sqrt(complex(E2^2 - 4*D2*F2)))/(2*D2)));

% --- LOOP 3 ---
a3 = L2_Loop3; b3 = L3_Loop3; c3 = L4_Loop3;
K1_L3=d/a3; K2_L3=d/c3; K3_L3=(a3^2-b3^2+c3^2+d^2)/(2*a3*c3); K4_L3=d/b3; K5_L3=(c3^2-d^2-a3^2-b3^2)/(2*a3*b3);
q_in_3 = q2_Cyan + pi;
A3=cos(q_in_3)-K1_L3-K2_L3*cos(q_in_3)+K3_L3; B3=-2*sin(q_in_3); C3=K1_L3-(K2_L3+1)*cos(q_in_3)+K3_L3;
D3=cos(q_in_3)-K1_L3+K4_L3*cos(q_in_3)+K5_L3; E3=-2*sin(q_in_3); F3=K1_L3+(K4_L3-1)*cos(q_in_3)+K5_L3;
q4_Brown = 2*atan(real((-B3 + sqrt(complex(B3^2 - 4*A3*C3)))/(2*A3)));
q3_Blue = 2*atan(real((-E3 + sqrt(complex(E3^2 - 4*D3*F3)))/(2*D3)));

% ==========================================
% SECTION 3: VELOCITY ANALYSIS 
% ==========================================
% Loop 1
w_Grey = (L3_Loop1 * w_Yellow * sin(q3 - q2_L1)) / (L4_Shared * sin(q4_L1 - q2_L1) );
w_Green = (L3_Loop1 * w_Yellow * sin(q4_L1 - q3)) / (L2_Loop1 * sin(q4_L1 - q2_L1) );
% Loop 2
w_Cyan = (L4_Shared * w_Grey * sin(q4_L1 - q3_Red)) / (L2_Loop2 * sin(q2_Cyan - q3_Red) );
w_Red = (L4_Shared * w_Grey * sin(q4_L1 - q2_Cyan)) / (L3_Loop2 * sin(q3_Red - q2_Cyan) );
% Loop 3
w_Brown = (L2_Loop3 * w_Cyan * sin(q_in_3 - q3_Blue)) / (L4_Loop3 * sin(q4_Brown - q3_Blue) );
w_Blue = (L2_Loop3 * w_Cyan * sin(q4_Brown - q_in_3)) / (L3_Loop3 * sin(q4_Brown - q3_Blue) );

% ==========================================
% SECTION 4: ACCELERATION ANALYSIS 
% ==========================================
% Loop 1
A1_c = -L2_Loop1*sin(q2_L1); B1_c = L4_Shared*sin(q4_L1); D1_c = L2_Loop1*cos(q2_L1); E1_c = -L4_Shared*cos(q4_L1);
C1_v = L2_Loop1*w_Green^2*cos(q2_L1) + L3_Loop1*w_Yellow^2*cos(q3) + L3_Loop1*alpha_Yellow*sin(q3) - L4_Shared*w_Grey^2*cos(q4_L1);
F1_v = L2_Loop1*w_Green^2*sin(q2_L1) + L3_Loop1*w_Yellow^2*sin(q3) - L3_Loop1*alpha_Yellow*cos(q3) - L4_Shared*w_Grey^2*sin(q4_L1);
det1 = A1_c*E1_c - B1_c*D1_c + 1e-12;
alp_Green = (C1_v*E1_c - B1_c*F1_v) / det1;
alp_Grey = (A1_c*F1_v - C1_v*D1_c) / det1;

% Loop 2
A2_c = -L2_Loop2*sin(q2_Cyan); B2_c = -L3_Loop2*sin(q3_Red); D2_c = L2_Loop2*cos(q2_Cyan); E2_c = L3_Loop2*cos(q3_Red);
C2_v = L2_Loop2*w_Cyan^2*cos(q2_Cyan) + L3_Loop2*w_Red^2*cos(q3_Red) - L4_Shared*w_Grey^2*cos(q4_L1) - L4_Shared*alp_Grey*sin(q4_L1);
F2_v = L2_Loop2*w_Cyan^2*sin(q2_Cyan) + L3_Loop2*w_Red^2*sin(q3_Red) - L4_Shared*w_Grey^2*sin(q4_L1) + L4_Shared*alp_Grey*cos(q4_L1);
det2 = A2_c*E2_c - B2_c*D2_c + 1e-12;
alp_Cyan = (C2_v*E2_c - B2_c*F2_v) / det2;
alp_Red = (A2_c*F2_v - C2_v*D2_c) / det2;

% Loop 3
A3_c = -L3_Loop3*sin(q3_Blue); B3_c = L4_Loop3*sin(q4_Brown); D3_c = L3_Loop3*cos(q3_Blue); E3_c = -L4_Loop3*cos(q4_Brown);
C3_v = L2_Loop3*w_Cyan^2*cos(q_in_3) + L2_Loop3*alp_Cyan*sin(q_in_3) + L3_Loop3*w_Blue^2*cos(q3_Blue) - L4_Loop3*w_Brown^2*cos(q4_Brown);
F3_v = L2_Loop3*w_Cyan^2*sin(q_in_3) - L2_Loop3*alp_Cyan*cos(q_in_3) + L3_Loop3*w_Blue^2*sin(q3_Blue) - L4_Loop3*w_Brown^2*sin(q4_Brown);
det3 = A3_c*E3_c - B3_c*D3_c + 1e-12;
alp_Blue = (C3_v*E3_c - B3_c*F3_v) / det3;
alp_Brown = (A3_c*F3_v - C3_v*D3_c) / det3;

% ==========================================
% SECTION 5: OUTPUT MAPPING (23 Ports)
% ==========================================
RO4x = d*cos(offset); RO4y = d*sin(offset);
Px = RO4x + 0.065 * cos(q4_Brown + offset);
Py = RO4y + 0.065 * sin(q4_Brown + offset);

xout = zeros(23,1);
% Angles (1-7)
xout(1:7) = rad2deg([q2_L1, q3_global_rad, q4_L1, q2_Cyan, q3_Red, q3_Blue, q4_Brown] + offset);
% Velocities (8-14)
xout(8:14) = [w_Green, w_Yellow, w_Grey, w_Cyan, w_Red, w_Blue, w_Brown];
% Accelerations (15-21)
xout(15:21) = [alp_Green, alpha_Yellow, alp_Grey, alp_Cyan, alp_Red, alp_Blue, alp_Brown];
% Trajectory (22-23)
xout(22) = Px; xout(23) = Py;

y = xout;
end
