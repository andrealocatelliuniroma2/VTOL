% Phd 25/26 : Locatelli Andrea

%% test variazione delta su angolo = pi/2

delta = 0.01;
theta_base = pi/2;

disp("sin(theta -+ delta):");
sin(theta_base-delta)
sin(theta_base+delta)

disp("cos(theta -+ delta):");
cos(theta_base-delta)
cos(theta_base+delta)

%% test variazione delta su angolo = 0

delta = 0.01;
theta_base = 0;

disp("sin(theta -+ delta):");
sin(theta_base-delta)
sin(theta_base+delta)

disp("cos(theta -+ delta):");
cos(theta_base-delta)
cos(theta_base+delta)

%% test allocazione

delta = 0;
theta_bar = pi/2-delta;
s_bar = sin(theta_bar) ;
c_bar = cos(theta_bar) ;
dmy = 0.6;
b_tilde = 0.005;

Mx_des = 2;
My_des = 1;
Mz_des = 1;
Fz_des = 1;

MF = [Mx_des;My_des;Mz_des;Fz_des];

Allocation_matrix = [-s_bar s_bar 0;s_bar s_bar -2;-c_bar*dmy -c_bar*dmy -b_tilde;s_bar s_bar 1];
%disp(Allocation_matrix)

invAllocation_matrix = pinv(Allocation_matrix);

omega=invAllocation_matrix*MF;
disp(omega)

%%

clear; clc;

dmy     = 0.6;
b_tilde = 0.005;

Mx_des = 1;
My_des = 2;
Mz_des = 1;
Fz_des = 2;

MF = [Mx_des; My_des; Mz_des; Fz_des];

delta_vec = linspace(deg2rad(-20), deg2rad(20), 5000);

omega_all = zeros(3, numel(delta_vec));

for k = 1:numel(delta_vec)
    delta = delta_vec(k);

    theta_bar = pi/2 - delta;
    s_bar = sin(theta_bar);
    c_bar = cos(theta_bar);

    A = [ -s_bar        s_bar        0;
           s_bar        s_bar       -2;
          -c_bar*dmy   -c_bar*dmy   -b_tilde;
           s_bar        s_bar       1 ];

    omega_all(:,k) = pinv(A) * MF;
end

% Indici in cui tutte le componenti di omega sono > 0
idx_ok = all(omega_all > 0, 1);

delta_ok = delta_vec(idx_ok);

delta_min = min(delta_ok);
delta_max = max(delta_ok);

fprintf('delta_min = %.6f rad (%.2f deg)\n', delta_min, rad2deg(delta_min));
fprintf('delta_max = %.6f rad (%.2f deg)\n', delta_max, rad2deg(delta_max));

%%
for delta = [delta_min, delta_max]
    theta_bar = pi/2 - delta;
    s_bar = sin(theta_bar);
    c_bar = cos(theta_bar);

    A = [ -s_bar        s_bar        0;
           s_bar        s_bar       -2;
          -c_bar*dmy   -c_bar*dmy   -b_tilde;
           s_bar        s_bar       1 ];

    omega = pinv(A)*MF;
    fprintf('\nDelta = %.6f rad\n', delta);
    disp(omega)
end

%%

%syms dmy k b dmx dtx omega1_s1 omega2_s2 omega1_c1 omega2_c2 omega3_s3 omega3_c3_s4 omega3_c3_c4 
syms omega1_s1 omega2_s2 omega1_c1 omega2_c2 omega3_s3 omega3_c3_s4 omega3_c3_c4 

dmx = 0.6;
dtx = -2*dmx;
ala_y = 0.4;
ala_x = 0.15;
dmy = (1/2)*ala_y;
k = 7*(10^-5); 
b= 0.005*k;

omega = [omega1_s1 ;omega2_s2 ;omega1_c1; omega2_c2 ;omega3_s3 ;omega3_c3_s4 ;omega3_c3_c4];

Matrice_allocazione = [0 0 k k 0 0 k;
                       0 0 0 0 0 k 0;
                       -k -k 0 0 -k 0 0;
                       -dmy*k dmy*k b -b 0 0 b;
                       dmx*k dmx*k 0 0 dtx*k b 0;
                       -b b -dmy*k dmy*k -b dtx*k 0];

% disp(Matrice_allocazione)
% disp(Matrice_allocazione*omega)

Matrice_allocazione_ridotta = Matrice_allocazione(:, 1:end-1);

% disp(Matrice_allocazione_ridotta);

invMatrice_allocazione_ridotta = inv(Matrice_allocazione_ridotta);

Des = [0 ;0 ;1;1;1;-1];

ris = invMatrice_allocazione_ridotta*Des;

omega1_2 = sqrt(ris(1)^2 +ris(3)^2);
omega2_2 = sqrt(ris(2)^2 +ris(4)^2);
theta1 = atan2(ris(1),ris(3));
theta2 = atan2(ris(2),ris(4));

% NB ris(6) = -omega3_2 * c3
%    ris(5) = omega3_2 * s3

omega3_2 = sqrt(ris(5)^2 +ris(6)^2);
theta3 = atan2(ris(5),-ris(6));

%%

syms k c1 c2 dmy

A = [k*(c1+c2) k*(c2-c1);dmy*k*(c2-c1) dmy*k*(c1+c2)];
disp(A)
inv(A)