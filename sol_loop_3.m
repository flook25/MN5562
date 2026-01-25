clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d) - Pink
L2 = 0.118; % Crank (a) - Cyan (Input)
L3 = 0.210; % Coupler (b) - Blue
L4 = 0.118; % Rocker (c) - Brown

a = L2;
b = L3;
c = L4;
d = L1;

% Ground Offset
offset_deg = 0.81;
offset = deg2rad(offset_deg);

% --- 2. Define K Constants (Forward Analysis: Theta 2 is Input) ---
K1 = d/a;
K2 = d/c;
K3 = (a^2 - b^2 + c^2 + d^2)/(2*a*c);
K4 = d/b;
K5 = (c^2 - d^2 - a^2 - b^2)/(2*a*b);

% ==========================================
% CASE 1: Open Circuit (Input = 17.944) -> Point UP
% ==========================================
% *** Changed to POSITIVE to force Cyan link ABOVE ground ***
q2d_open = 17.944; 
q2_1 = deg2rad(q2d_open) - offset;

% --- Calculate Coefficients for Theta 4 ---
A1 = cos(q2_1) - K1 - K2*cos(q2_1) + K3;
B1 = -2*sin(q2_1);
C1 = K1 - (K2+1)*cos(q2_1) + K3;

% --- Calculate Coefficients for Theta 3 ---
D1 = cos(q2_1) - K1 + K4*cos(q2_1) + K5;
E1 = -2*sin(q2_1);
F1 = K1 + (K4-1)*cos(q2_1) + K5;

% --- Solve (Standard Minus Sqrt for Open) ---
q4_open = 2*atan((-B1 - sqrt(B1^2 - 4*A1*C1))/(2*A1));
q3_open = 2*atan((-E1 - sqrt(E1^2 - 4*D1*F1))/(2*D1));

q4_open_d = rad2deg(q4_open) + offset_deg;
q3_open_d = rad2deg(q3_open) + offset_deg;


% ==========================================
% CASE 2: Crossed Circuit (Input = 92.2333) -> Point UP
% ==========================================
% *** Changed to POSITIVE to force Cyan link ABOVE ground ***
q2d_cross
