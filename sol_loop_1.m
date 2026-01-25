clear all
close all
clc

% --- 1. Define Parameters (Unit: meters) ---
L1 = 0.210; % Ground (d)
L2 = 0.180; % Crank (a)
L3 = 0.180; % Coupler (b)
L4 = 0.118; % Rocker (c)

a = L2; b = L3; c = L4; d = L1;

% --- 2. Input Parameter & Coordinate Transform ---
offset_deg = 0.81;
offset = deg2rad(offset_deg);

q3_global = 19.94; % Input Theta 3 (Global)
q3 = deg2rad(q3_global) - offset; % Local Theta 3

% --- 3. Solve for Theta 4 (Algebraic Method) ---
% Equation: P4*cos(q4) + Q4*sin(q4) + R4 = 0
P4 = 2*c*(d - b*cos(q3));
Q4 = -2*b*c*sin(q3);
R4 = d^2 + b^2 + c^2 - a^2 - 2*d*b*cos(q3);

% Quadratic Coefficients (A*t^2 + B*t + C = 0)
A_q4 = R4 - P4;
B_q4 = 2*Q4;
C_q4 = R4 + P4;

% Two solutions for q4
q4_sol1 = 2*atan((-B_q4 + sqrt(B_q4^2 - 4*A_q4*C_q4))/(2*A_q4));
q4_sol2 = 2*atan((-B_q4 - sqrt(B_q4^2 - 4*A_q4*C_q4))/(2*A_q4));

% --- 4. Solve for Theta 2 (Algebraic Method) ---
% Equation: P2*cos(q2) + Q2*sin(q2) + R2 = 0
P2 = 2*a*(b*cos(q3) - d);
Q2 = 2*a*b*sin(q3);
R2 = a^2 + b^2 + d^2 - c^2 - 2*b*d*cos(q3);

% Quadratic Coefficients (D*t^2 + E*t + F = 0)
D_q2 = R2 - P2;
E_q2 = 2*Q2;
F_q2 = R2 + P2;

% Two solutions for q2
q2_sol1 = 2*atan((-E_q2 - sqrt(E_q2^2 - 4*D_q2*F_q2))/(2*D_q2));
q2_sol2 = 2*atan((-E_q2 + sqrt(E_q2^2 - 4*D_q2*F_q2))/(2*D_q2));

% --- 5. Validating and Pairing Solutions (CRITICAL STEP) ---
% We have 2 q4's and 2 q2's. We must check which pairs close the loop.
% Loop Eq: a*exp(jq2) + b*exp(jq3) - c*exp(jq4) - d = 0
possible_q2 = [q2_sol1, q2_sol2];
possible_q4 = [q4_sol1, q4_sol2];

valid_pairs = []; % To store [index_q2, index_q4]

for i = 1:2
    for j = 1:2
        t2 = possible_q2(i);
        t4 = possible_q4(j);
        % Vector Loop Error Check
        loop_err = abs(a*exp(1j*t2) + b*exp(1j*q3) - c*exp(1j*t4) - d);
        
        if loop_err < 1e-4 % Tolerance
            valid_pairs = [valid_pairs; i, j];
        end
    end
end

% Assign Valid Configurations (Config 1 & Config 2)
if size(valid_pairs, 1) < 2
    error('Could not find valid assembly configurations.');
end

% Config 1
idx2_1 = valid_pairs(1,1);
idx4_1 = valid_pairs(1,2);
q2_config1 = possible_q2(idx2_1);
q4_config1 = possible_q4(idx4_1);

% Config 2
idx2_2 = valid_pairs(2,1);
idx4_2 = valid_pairs(2,2);
q2_config2 = possible_q2(idx2_2);
q4_config2 = possible_q4(idx4_2);

% --- 6. Convert to Global Degrees ---
q21d = rad2deg(q2_config1) + offset_deg;
q41d = rad2deg(q4_config1) + offset_deg;

q22d = rad2deg(q2_config2) + offset_deg;
q42d = rad2deg(q4_config2) + offset_deg;

% --- 7. Plotting (Quiver Only) ---
% Ground Vector
RO4O2 = d*exp(1j*offset);
RO4O2x = real(RO4O2); RO4O2y = imag(RO4O2);

% -- Config 1 --
RA1 = a*exp(1j*(q2_config1 + offset));
RBA = b*exp(1j*(q3 + offset));
RBO4_1 = c*exp(1j*(q4_config1 + offset));

RA1x = real(RA1); RA1y = imag(RA1);
RBAx = real(RBA); RBAy = imag(RBA);
RBO4_1x = real(RBO4_1); RBO4_1y = imag(RBO4_1);

% -- Config 2 --
RA2 = a*exp(1j*(q2_config2 + offset));
RBO4_2 = c*exp(1j*(q4_config2 + offset));

RA2x = real(RA2); RA2y = imag(RA2);
RBO4_2x = real(RBO4_2); RBO4_2y = imag(RBO4_2);

% Plot Figure 1: Config 1
figure(1);
title('Configuration 1 (Valid Assembly)');
hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'black', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Ground
quiver(0,0, RA1x, RA1y, 0, 'red', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Crank
quiver(RA1x, RA1y, RBAx, RBAy, 0, 'blue', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Coupler
quiver(RO4O2x, RO4O2y, RBO4_1x, RBO4_1y, 0, 'green', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Rocker
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Plot Figure 2: Config 2
figure(2);
title('Configuration 2 (Valid Assembly)');
hold on;
quiver(0,0, RO4O2x, RO4O2y, 0, 'black', 'LineWidth', 3, 'MaxHeadSize', 0.5); % Ground
quiver(0,0, RA2x, RA2y, 0, 'red', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Crank
quiver(RA2x, RA2y, RBAx, RBAy, 0, 'blue', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Coupler
quiver(RO4O2x, RO4O2y, RBO4_2x, RBO4_2y, 0, 'green', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Rocker
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');

% Display Results
disp('--- Results (Corrected with Loop Check) ---');
disp(['Input Theta 3 (Global): ', num2str(q3_global)]);
disp(' ');
disp('Configuration 1:');
disp(['  Theta 2: ', num2str(q21d), ' deg']);
disp(['  Theta 4: ', num2str(q41d), ' deg']);
disp(' ');
disp('Configuration 2:');
disp(['  Theta 2: ', num2str(q22d), ' deg']);
disp(['  Theta 4: ', num2str(q42d), ' deg']);
