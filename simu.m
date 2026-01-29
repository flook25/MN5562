function y = fcn(u1,u2,u3,u4,u5,u6,u7)
%u1: w1
%u2:theta2 in rad
%u3: theta3 in rad
%u4: theta4 in rad

L1 = 0.1; %length of Link1 d
L2 = 0.04; %length of Link2 a
L3 = 0.12; %length of Link3 c
L4 = 0.08; %length of Link4 d
a = L2;
b = L3;
c = L4;
d = L1;

alpha2 = u1
%w2 = u1;
%q2d = rad2deg(u2);
w2 = u2;
w32 = u3;
w42 = u4;
q2 = u5; % radian angle
q32 = u6;
q42 = u7;

RA = a*exp(j*q2);
%RBA1 = b*exp(j*q31); %q31 Crossed circuit
RBA2 = b*exp(j*q32); %q32 Open circuit

%RB1 = RA + RBA1; %Position vector for Crossed circuit
RB2 = RA + RBA2; %Position vector for Open circuit

RAx = real(RA); 
RAy = imag(RA);

%RBA1x = real(RBA1); %for Crossed circuit
%RBA1y = imag(RBA1); %for Crossed circuit

RBA2x = real(RBA2);
RBA2y = imag(RBA2);

%RB1x = real(RB1); %Position Bx for Crossed circuit
%RB1y = imag(RB1); %Position By for Crossed circuit
RB2x = real(RB2); %Position Bx for Open circuit
RB2y = imag(RB2); %Position By for Open circuit

VA = j*a*w2*exp(j*q2);
VAx = real(VA);
VAy = imag(VA);
VBA2 = j*b*w32*exp(j*q32);
VBA2x = real(VBA2);
VBA2y = imag(VBA2);

%VB2 = j*c*w42*exp(j*q42);
VB =  VA + VBA2; 
VBx = real(VB);
VBy = imag(VB);
%VB2x = real(VB2);
%VB2y = imag(VB2);

%alpha2 = 15; %25 rad/sec^2

A = c*sin(q42); B = b*sin(q32);
C = a*alpha2*sin(q2)+a*w2^2*cos(q2)+b*w32^2*cos(q32)-c*w42^2*cos(q42);
D = c*cos(q42); E = b*cos(q32);
F = a*alpha2*cos(q2)-a*w2^2*sin(q2)-b*w32^2*sin(q32)+c*w42^2*sin(q42);

alpha42 = (C*E-B*F)/(A*E-B*D);
alpha32 = (C*D-A*F)/(A*E-B*D);

AA = a*alpha2*j*exp(j*q2)-a*w2^2*exp(j*q2);% polar coordinate 
AAx = real(AA); AAy = imag(AA);

ABA = b*alpha32*j*exp(j*q32)-b*w32^2*exp(j*q32);
ABAx = real(ABA); ABAy = imag(ABA);

AB = c*alpha42*j*exp(j*q42) - c*w42^2*exp(j*q42);
ABx = real(AB); ABy = imag(AB);

xout = zeros(20,1);
xout(1) = alpha32; 
xout(2) = alpha42; 
xout(3) = RAx; 
xout(4) = RAy; 
xout(5) = RBA2x; 
xout(6) = RBA2y; 
xout(7) = RB2x; 
xout(8) = RB2y; 
xout(9) = VAx; 
xout(10) = VAy; 
xout(11) = VBA2x;
xout(12) = VBA2y;
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
