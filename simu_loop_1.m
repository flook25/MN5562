function y = fcn(u1, u2, u3, u4, u5, u6, u7)
% u1: alpha_Yellow_input (0.003)
% u2: w_Yellow (input velocity)
% u3: q3_global (input angle in rad)
% u4-u7: (Reserved for other states if wired from integrators)

% ==========================================
% SECTION 1: PARAMETERS & INPUTS
% ==========================================
L1 = 0.210; d = L1;
a1 = 0.180; b1 = 0.180; c1 = 0.118; % Loop 1 (Green, Yellow, Grey)
a2 = 0.118; b2 = 0.210;             % Loop 2 (Cyan, Red)
a3 = 0.118; b3 = 0.210; c3 = 0.118; % Loop 3 (Cyan Up, Blue, Brown)
offset = 0.014137; % 0.81 degree in rad

alpha3 = u1; % Input Alpha (Yellow)
w3 = u2;     % Input Omega (Yellow)
q3_glob = u3; 
q3 = q3_glob - offset; % Local theta 3 for calculation

% ==========================================
% SECTION 2: POSITION & VELOCITY & ACCELERATION (LOOP 1)
% ==========================================
% --- Position ---
K1=d/b1; K2=d/c1; K3=(b1^2-a1^2+c1^2+d^2)/(2*b1*c1); K4=d/a1; K5=(c1^2-d^2-b1^2-a1^2)/(2*b1*a1);
A_p=cos(q3)-K1-K2*cos(q3)+K3; B_p=-2*sin(q3); C_p=K1-(K2+1)*cos(q3)+K3;
D_p=cos(q3)-K1+K4*cos(q3)+K5; E_p=-2*sin(q3); F_p=K1+(K4-1)*cos(q3)+K5;
q4 = 2*atan2((-B_p - sqrt(B_p^2 - 4*A_p*C_p)), 2*A_p); % Grey Angle
q2 = 2*atan2((-E_p - sqrt(E_p^2 - 4*D_p*F_p)), 2*D_p); % Green Angle

% --- Velocity ---
w4 = (b1 * w3 * sin(q3 - q2)) / (c1 * sin(q4 - q2)); % Grey Vel
w2 = (b1 * w3 * sin(q4 - q3)) / (a1 * sin(q4 - q2)); % Green Vel

% --- Acceleration (Analytical Method [cite: 171-175]) ---
A_a = -a1*sin(q2); B_a = c1*sin(q4); D_a = a1*cos(q2); E_a = -c1*cos(q4);
C_v = a1*w2^2*cos(q2) + b1*w3^2*cos(q3) + b1*alpha3*sin(q3) - c1*w4^2*cos(q4);
F_v = a1*w2^2*sin(q2) + b1*w3^2*sin(q3) - b1*alpha3*cos(q3) - c1*w4^2*sin(q4);
alpha2 = (C_v*E_a - B_a*F_v) / (A_a*E_a - B_a*D_a); % Green Alpha
alpha4 = (A_a*F_v - C_v*D_a) / (A_a*E_a - B_a*D_a); % Grey Alpha

% ==========================================
% SECTION 3: POINT P TRAJECTORY (Brown Link)
% ==========================================
% คำนวณตำแหน่งจุด P (65mm บน Brown link ซึ่งหมุนตามมุม Grey ใน Loop 1)
RO4x = d*cos(offset); RO4y = d*sin(offset);
Px = RO4x + 0.065 * cos(q4 + offset);
Py = RO4y + 0.065 * sin(q4 + offset);

% ==========================================
% SECTION 4: OUTPUT MAPPING (20 CHANNELS)
% ==========================================
xout = zeros(20,1);

% 1-3: Angular Accelerations (alpha) - [cite: 173, 231-233]
xout(1) = alpha2; % Green
xout(2) = alpha3; % Yellow (Input)
xout(3) = alpha4; % Grey

% 4-6: Angles in Degrees (for Plotting 1)
xout(4) = rad2deg(q2 + offset); % Green
xout(5) = rad2deg(q3_glob);     % Yellow
xout(6) = rad2deg(q4 + offset); % Grey

% 7-9: Angular Velocities (omega) - [cite: 327]
xout(7) = w2; % Green
xout(8) = w3; % Yellow
xout(9) = w4; % Grey

% 10-11: Point P Trajectory (for Plotting 4)
xout(10) = Px;
xout(11) = Py;

% 12-14: (Reserved for Loop 2/3 if needed)
xout(12) = 0; 

% 15-20: Linear Accelerations Components [cite: 247-257]
% AAx, AAy (Green Tip)
xout(15) = -a1*alpha2*sin(q2+offset) - a1*w2^2*cos(q2+offset);
xout(16) =  a1*alpha2*cos(q2+offset) - a1*w2^2*sin(q2+offset);
% ABx, ABy (Grey Tip)
xout(19) = -c1*alpha4*sin(q4+offset) - c1*w4^2*cos(q4+offset);
xout(20) =  c1*alpha4*cos(q4+offset) - c1*w4^2*sin(q4+offset);

y = xout;
end
