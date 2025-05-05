clc;
clear;

%forward positionen
theta1_target = 90;
theta2_target = 30;
theta3_target = 86;
theta4_target = 25;


%grad -> rad
theta1 = deg2rad(theta1_target);
theta2 = deg2rad(theta2_target);
theta3 = deg2rad(theta3_target);
theta4 = deg2rad(theta4_target);

% inverse längen
L1 = 127.026;
L2 = 119.980;
L3 = 122.308;
L4 = 129.823;

%forward kinematic
phi = theta2 + theta3 + theta4;

a = L2 * cos(theta2) + L3 * cos(theta2 + theta3);
b = L2 * sin(theta2) + L3 * sin(theta2 + theta3);

x = (a + L4 * cos(phi)) * cos(theta1);
y = (a + L4 * cos(phi)) * sin(theta1);
z = L1 + b + L4 * sin(phi);

fprintf("forward kinematic:\n");
fprintf("x = %.2f\n", x);
fprintf("y = %.2f\n", y);
fprintf("z = %.2f\n", z);
fprintf("phi = %.2f°\n\n", rad2deg(phi));

%inverse kinematic --> begin
phi_inv = phi;

theta1_ik = atan2(y, x);

% SPIEGELUNG BC SERVOS 0-180° keypoint
if rad2deg(theta1_ik) < 0 || rad2deg(theta1_ik) > 180
    theta1_ik = mod(theta1_ik + pi, 2*pi);  % z.B. 270° → 90°
end

A = x - L4 * cos(phi_inv) * cos(theta1_ik);
B = y - L4 * cos(phi_inv) * sin(theta1_ik);
C = z - L1 - L4 * sin(phi_inv);
R = sqrt(A^2 + B^2);

%kosinussatz
D = (R^2 + C^2 - L2^2 - L3^2) / (2 * L2 * L3);
D = max(min(D, 1), -1);  % Begrenzen

%ellbow down
theta3_1 = acos(D);
a1 = L3 * sin(theta3_1);
b1 = L2 + L3 * cos(theta3_1);
theta2_1 = atan2(C, R) - atan2(a1, b1);
theta4_1 = phi_inv - theta2_1 - theta3_1;

%ellbow up
theta3_2 = -acos(D);
a2 = L3 * sin(theta3_2);
b2 = L2 + L3 * cos(theta3_2);
theta2_2 = atan2(C, R) - atan2(a2, b2);
theta4_2 = phi_inv - theta2_2 - theta3_2;

%%winkel normalisieren
normalize = @(angle_rad) mod(rad2deg(angle_rad) + 360, 360);

%ellbow down solution
theta1_1_deg = normalize(theta1_ik);
theta2_1_deg = normalize(theta2_1);
theta3_1_deg = normalize(theta3_1);
theta4_1_deg = normalize(theta4_1);

%ellbow up solution
theta1_2_deg = normalize(theta1_ik);
theta2_2_deg = normalize(theta2_2);
theta3_2_deg = normalize(theta3_2);
theta4_2_deg = normalize(theta4_2);

%0-180°?
isValid1 = all([theta1_1_deg, theta2_1_deg, theta3_1_deg, theta4_1_deg] <= 180);
isValid2 = all([theta1_2_deg, theta2_2_deg, theta3_2_deg, theta4_2_deg] <= 180);

if isValid1
    fprintf("ellbow down:\n");
    fprintf("theta1 = %.2f°\n", theta1_1_deg);
    fprintf("theta2 = %.2f°\n", theta2_1_deg);
    fprintf("theta3 = %.2f°\n", theta3_1_deg);
    fprintf("theta4 = %.2f°\n\n", theta4_1_deg);
else
    fprintf("ellbow down geht nicht da: 0°, 180°\n\n");
end

if isValid2
    fprintf("ellbow up:");
    fprintf("theta1 = %.2f°\n", theta1_2_deg);
    fprintf("theta2 = %.2f°\n", theta2_2_deg);
    fprintf("theta3 = %.2f°\n", theta3_2_deg);
    fprintf("theta4 = %.2f°\n\n", theta4_2_deg);
else
    fprintf("ellbow up geht nicht da außerhalb von 0° - 180°\n\n");
end

