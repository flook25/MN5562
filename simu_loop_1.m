function y = fcn(u1,u2,u3,u4,u5,u6,u7)
% Inputs mapping สำหรับกลไกของคุณ (Loop 1):
% u1: alpha_Yellow (Input Angular Acceleration = 0.003)
% u2: w_Green (ความเร็วเชิงมุมลิ้งค์ 2)
% u3: w_Yellow (ความเร็วเชิงมุมลิ้งค์ 3 - Input = 0.03)
% u4: w_Grey (ความเร็วเชิงมุมลิ้งค์ 4)
% u5: q_Green (มุมลิ้งค์ 2) ในหน่วย rad
% u6: q_Yellow (มุมลิ้งค์ 3) ในหน่วย rad
% u7: q_Grey (มุมลิ้งค์ 4) ในหน่วย rad

% ==========================================
% 1. PARAMETERS (Loop 1: Green-Yellow-Grey) [cite: 265, 558-559, 741-742]
% ==========================================
L1 = 0.210; % Ground (d)
L2 = 0.180; % Green (a)
L3 = 0.180; % Yellow (b)
L4 = 0.118; % Grey (c)
offset = 0.014137; % 0.81 degree in radian

a = L2; b = L3; c = L4; d = L1;

% กำหนดค่าอินพุตตาม u1-u7
alpha_Yellow = u1; 
w2 = u2; w3 = u3; w4 = u4;
q2 = u5; q3 = u6; q4 = u7;

% ==========================================
% 2. POSITION VECTORS [cite: 568-574, 608-614, 820-826]
% ==========================================
RA = a*exp(1i*q2);
RAx = real(RA); RAy = imag(RA);

RBA = b*exp(1i*q3);
RBAx = real(RBA); RBAy = imag(RBA);

RB = RA + RBA;
RBx = real(RB); RBy = imag(RB);

% ==========================================
% 3. VELOCITY VECTORS [cite: 670-677, 688-695, 936-942]
% ==========================================
VA = 1i*a*w2*exp(1i*q2);
VAx = real(VA); VAy = imag(VA);

VBA = 1i*b*w3*exp(1i*q3);
VBAx = real(VBA); VBAy = imag(VBA);

VB = VA + VBA;
VBx = real(VB); VBy = imag(VB);

% ==========================================
% 4. ACCELERATION ANALYSIS (Analytical Method) [cite: 171-175, 231-233, 335-345]
% ==========================================
% ตัวแปรตามสไลด์ (A, B, D, E) [cite: 172, 336, 340]
A_acc = c*sin(q4); 
B_acc = b*sin(q3); 
D_acc = c*cos(q4); 
E_acc = b*cos(q3);

% พจน์คงที่ RHS (C, F) [cite: 175, 222-223, 337-343]
% โดยที่ alpha_input (u1) อยู่ที่ลิ้งค์เหลือง (ลิ้งค์ 3)
C_val = a*alpha_input_not_used*0 + a*w2^2*cos(q2) + b*w3^2*cos(q3) + b*alpha_Yellow*sin(q3) - c*w4^2*cos(q4);
% หมายเหตุ: ในโจทย์นี้ alpha_Green (ลิ้งค์ 2) เป็น Unknown จึงย้ายไปฝั่งแก้สมการ
% เราต้องจัดรูปสมการใหม่ตามพจน์ u1 (alpha3) ที่เรารู้ค่า

% ปรับสูตรแก้หา alpha2 (Green) และ alpha4 (Grey) ตามหลักการกำจัดตัวแปรในสไลด์ [cite: 154-159, 205-208]
A_mod = -a*sin(q2); B_mod = c*sin(q4);
D_mod = a*cos(q2);  E_mod = -c*cos(q4);

C_mod = a*w2^2*cos(q2) + b*w3^2*cos(q3) + b*alpha_Yellow*sin(q3) - c*w4^2*cos(q4);
F_mod = a*w2^2*sin(q2) + b*w3^2*sin(q3) - b*alpha_Yellow*cos(q3) - c*w4^2*sin(q4);

alpha2 = (C_mod*E_mod - B_mod*F_mod) / (A_mod*E_mod - B_mod*D_mod);
alpha4 = (A_mod*F_mod - C_mod*D_mod) / (A_mod*E_mod - B_mod*D_mod);

% ==========================================
% 5. ACCELERATION VECTORS [cite: 247-257, 355-367]
% ==========================================
AA = (1i*alpha2*a - a*w2^2)*exp(1i*q2);
AAx = real(AA); AAy = imag(AA);

ABA = (1i*alpha_Yellow*b - b*w3^2)*exp(1i*q3);
ABAx = real(ABA); ABAy = imag(ABA);

AB = AA + ABA;
ABx = real(AB); ABy = imag(AB);

% ==========================================
% 6. OUTPUT MAPPING (Exact Format: 20 Channels)
% ==========================================
xout = zeros(20,1);
xout(1) = alpha2; % alpha ของ Green
xout(2) = alpha4; % alpha ของ Grey
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
