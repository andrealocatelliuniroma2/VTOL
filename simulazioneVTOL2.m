function x_dot = simulazioneVTOL2(x,params)

% check parametri

paramFlag = 0;

if paramFlag == 1
    % === Stampa dei parametri ===
    fprintf('\n========= PARAMETRI VTOL =========\n');
    fprintf('Massa (m):            %.2f kg\n', params.m);
    fprintf('Gravità (g):          %.2f m/s^2\n', params.g);
    fprintf('Thrust coeff. (k):    %.2f\n', params.k);
    fprintf('Drag coeff. (C_d):    %.2f\n', params.C_d);
    fprintf('Lift coeff. (C_l):    %.2f\n', params.C_l);
    fprintf('Densità aria (rho):   %.2f kg/m^3\n', params.rho);
    fprintf('Dimensioni ala (ala_x): %.2f m\n', params.ala_x);
    fprintf('Dimensioni ala (ala_y): %.2f m\n', params.ala_y);
    fprintf('Superficie alare (s): %.2f m^2\n', params.s);
    fprintf('Velocità aria:        %.2f m/s\n', params.v_air);
    fprintf('b (torque/thrust): %.2f\n', params.b);
    fprintf('\n-- Distanze rotori rispetto al centro di massa --\n');
    disp('r_th_w_dx ='); disp(params.r_th_w_dx);
    disp('r_th_w_sx ='); disp(params.r_th_w_sx);
    disp('r_th_tail ='); disp(params.r_th_tail);

    fprintf('-- Distanze forze aerodinamiche --\n');
    disp('r_aerodyn_w_dx ='); disp(params.r_aerodyn_w_dx);
    disp('r_aerodyn_w_sx ='); disp(params.r_aerodyn_w_sx);
    %disp('r_aerodyn_tail ='); disp(params.r_aerodyn_tail);

    fprintf('-- Matrici di inerzia --\n');
    disp('I_body ='); disp(params.I_body);
    disp('I_rotor_w_dx ='); disp(params.I_rotor_w_dx);
    disp('I_rotor_w_sx ='); disp(params.I_rotor_w_sx);
    disp('I_rotor_tail ='); disp(params.I_rotor_tail);
    fprintf('===================================\n\n');

end

% stati e controllo

phi = x(7);
theta = x(8);
psi = x(9);

p = x(10);
q = x(11);
r = x(12); 

V_body = [x(4);x(5);x(6)]; % velocità nel body frame
Omega_body = [p;q;r]; % velocità angolare body


%% Controllo
% if x(2)<-0.45
%    params.switch = 1;
% end
u = controlloVTOL_v2(params,x);


%% dinamica tilt rotor

simbolico = 0;

% rotori anteriori
% zeta = 0.8;
% omega_n = 2*pi*2;

zeta = 0.9;
omega_n = 2*pi*2;

% tail rotor
zeta_tail = 0.8;
omega_n_tail = 2*pi*4;

% dinamica eliche rotori
zeta_rotor = 0.9;
omega_n_rotor = 2*pi*15;

if simbolico == 1

    syms zeta omega_n zeta_tail omega_n_tail zeta_rotor omega_n_rotor

end

theta1_des = u(4);
theta2_des = u(5);
theta3_des = u(6);
theta4_des = u(7);

%% matrici di rotazione e trasformazione

R = matriceRotazione(phi,theta,psi); % V_global = R*V_body 
J = matriceJ(phi,theta,psi); % matrice di trasformazione  : OmegaVtol_body (p,q,r) -> Omega_global (phi_dot,theta_dot,psi_dot)

%% wind frame (NB:questo blocco di codice è anche nel controllo e nelle condizioni iniziali del main)
Va = sqrt((x(4)^2)+(x(5)^2)+(x(6)^2)); % airspeed 

%%%%%%%%%%%%%%%%%%%%%%%%%%% check velocità %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Va_min = 0.001;  % sotto questa soglia è come se non mi muovessi
if Va < Va_min
    alpha = 0;
    beta  = 0;
else

    alpha = atan2(-x(6),x(4)); % angle of attack (il - dipende da NED)
    beta = atan2(x(5),sqrt((x(4)^2)+(x(6)^2))); % sideslip angle

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Rwb = matriceRotazioneWingToBodyFrame(alpha,beta);

%% forze aerodinamiche

%%%%%%%%%%%%%%%%% VOLO VERTICALE %%%%%%%%%%%%%%%%%%%%%%%%

% nuovo metodo : aerodinamica corpo
F_aeroBody = Drag_body(params.C_d_x,params.C_d_y,params.C_d_z, params.rho,params.s_body_x,params.s_body_y,params.s_body_z,x(4),x(5),x(6));

% vecchio metodo : aerodinamica ali
% tiene conto solo di portanza e drag lungo x delle ali (va bene per volo verticale)
%F_aeroWing = F_aerodyn_wing(params.C_l,params.C_d,0, params.rho ,x(4),x(6), params.s);

% nel caso di volo verticale va "riconsiderato coefficente di lift"

alpha_abs = abs(alpha);
alpha1 = deg2rad(45);   % inizio perdita di validità del modello di portanza
alpha2 = deg2rad(85);   % portanza trascurabile

% ottengo una coordinata normalizzata tra 0 e 1 in base al valore
% dell'attuale angolo di attacco nell'intervallo considerato
t = (alpha_abs - alpha1) / (alpha2 - alpha1);
t = min(max(t, 0), 1);
% funzione che decresce (max se t = 0 e min se t = 1)
n = 1 - (3*t^2 - 2*t^3);
% decadimento del coeff. di lift
CL_eff = n * params.C_l;

F_aeroWing = F_aero_wing(CL_eff,params.C_d,params.C_y, params.rho, params.s,Va,Rwb);


% %%%%%%%%%%%%%%%%% VOLO ORIZZONTALE %%%%%%%%%%%%%%%%%%%%%%%%
% 
% % nuovo metodo : aerodinamica corpo
% F_aeroBody = Drag_body(params.C_d_x,params.C_d_y,params.C_d_z, params.rho,params.s_body_x,params.s_body_y,params.s_body_z,x(4),x(5),x(6));
% 
% % nuovo metodo : aerodinamica ali
% F_aeroWing = F_aero_wing(params.C_l,params.C_d,params.C_y, params.rho, params.s,Va,Rwb);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% eq. forze BODY FRAME

% GRAVITA'

F_g_body = F_grav(phi , theta , psi , params.m ,params.g); % body frame

% disp("F_g_body = ");
% disp(F_g_body);

% THRUST 

input_thrust = [params.k*x(21)^2;params.k*x(23)^2;params.k*x(25)^2;x(13);x(15);x(17);x(19)];
%input_thrust = [params.k*x(21)^2;params.k*x(23)^2;params.k*x(25)^2;u(4);u(5);x(17);x(19)];
%input_thrust = [params.k*u(1)^2;params.k*u(2)^2;params.k*u(3)^2;x(13);x(15);x(17);x(19)];
%input_thrust = [params.k*u(1)^2;params.k*u(2)^2;params.k*u(3)^2;u(4);u(5);u(6);u(7)];
%input_thrust = [params.k*u(1)^2;params.k*u(2)^2;params.k*u(3)^2;u(4);u(5);x(17);x(19)];

F_th_body = F_thrust(input_thrust); % body frame

% disp("F_th_body = ");
% disp(F_th_body);

% forze aerodinamiche nel body frame (DRAG+LIFT) (VECCHIO)
%F_aero_body = F_aerodyn_wing(params.C_l,params.C_d,params.C_d_z, params.rho ,x(4),x(6), params.s);

%F_aeroWing = F_aerodyn_wing(params.C_l,params.C_d,0, params.rho ,x(4),x(6), params.s);
F_aero_body = F_aeroWing + F_aeroBody;

% disp("F_aero_body = ");
% disp(F_aero_body);

% CORIOLIS

F_cor = F_Coriolis(Omega_body,V_body,params.m); % termine di Coriolis , sono nel body frame

% disp("F_Coriolis = ");
% disp(F_cor);


% FORZE TOTALI

F_tot_body = F_g_body+F_th_body+F_aero_body-F_cor; % body frame

if abs(F_tot_body(1)) >= 0.0001
    test = 1;
end

%F_tot_body(2)=0;

% disp("F_tot_body = ");
% disp(F_tot_body);


%% eq. Momenti


M_gyro_body = MomentGyroBody(params.I_body,Omega_body);
%M_gyro_body = [0;0;0];

% disp("M_gyroBody = ");
% disp(M_gyro_body);

% per il momento torcente
Iz1 = params.I_rotor_w_dx(3,3);
Iz2 = params.I_rotor_w_sx(3,3);
Iz3 = params.I_rotor_tail(3,3);


M_th = M_thrust_2(input_thrust,params.r_th_w_dx,params.r_th_w_sx,params.r_th_tail,params.b,params.k,x(22),x(24),x(26),Iz1,Iz2,Iz3);
%M_th = M_thrust_noTorc(input_thrust,params.r_th_w_dx,params.r_th_w_sx,params.r_th_tail,params.b,params.k,x(22),x(24),x(26),Iz1,Iz2,Iz3);


% disp("M_th = ");
% disp(M_th);

M_aero = MomentAero(params.r_aerodyn_w_dx,params.r_aerodyn_w_sx,params.C_l,params.C_d,0, params.rho ,x(4),x(6), params.s);

% disp("M_aero = ");
% disp(M_aero);

% momento giroscopico del til dei dei rotori 
Omega_rotor_w_dx = [0;x(14);0];
Omega_rotor_w_sx = [0;x(16);0];
Omega_rotor_tail = [0;x(18);x(20)];

input_thrust_gyro = [x(21);x(23);x(25);x(13);x(15);x(17);x(19)];
M_gyro_tilt = M_tilt_rotor(input_thrust_gyro,params.I_rotor_w_dx,params.I_rotor_w_sx,params.I_rotor_tail,Omega_rotor_w_dx , Omega_rotor_w_sx , Omega_rotor_tail);
%M_gyro_tilt =[0;0;0];

% disp("M_gyro_tilt = ");
% disp(M_gyro_tilt);


alpha0x =1;
alpha1x =1;
alpha0y =1;
alpha1y=1;
alpha0z =1;
alpha1z =1;

M_stab_pinna = [-x(10)*alpha0x+alpha1x*x(5)^2;0;-x(12)*(alpha0z+alpha1z*x(5)^2)];
% con variante in assenza di rotore anteriore
%[-x(10)*alpha0x-alpha1x*(x(5)^2 +x(6)^2);-x(11)*(alpha0y+alpha1y*x(6)^2);-x(12)*(alpha0z+alpha1z*x(5)^2)];
%M_stab_pinna = [0;0;0];

%M_tot1 = -M_gyro_body + M_th + M_aero +M_gyro_tilt; 
M_tot = -M_gyro_body + M_th + M_aero +M_gyro_tilt+M_stab_pinna; 

%M_tot(2)=0;
% M_tot(3)=0;
% M_tot(1)=0;

%M_tot(abs(M_tot) < 1e-10) = 0;

if M_tot(3) ~= 0 || abs(M_tot(2))>1e-5 || M_tot(1)~=0
    debug = 1;
end

if M_tot(1) ~= 0
    debug = 1;
end

% azione integrale

e_PI_1 = u(8);
e_PI_2 = u(9);

if u(8)==0
    test=1;
end
if u(9)==0
    test=1;
end
if x(4)==25
    test=1;
end


x123_dot = R*V_body;
x1_dot = x123_dot(1);
x2_dot = x123_dot(2);
x3_dot = x123_dot(3);

x4_dot = (1/params.m)*F_tot_body(1);
x5_dot = (1/params.m)*F_tot_body(2);
x6_dot = (1/params.m)*F_tot_body(3);

x789_dot = J*Omega_body;
x7_dot = x789_dot(1);
x8_dot = x789_dot(2);
x9_dot = x789_dot(3);

x_101112_dot = inv(params.I_body)*M_tot;
x10_dot = x_101112_dot(1);
x11_dot = x_101112_dot(2);
x12_dot = x_101112_dot(3);

global p_dot_global q_dot_global r_dot_global
p_dot_global = x10_dot;
q_dot_global = x11_dot;
r_dot_global = x12_dot;


x13_dot = x(14);
x14_dot = -2*zeta*omega_n*x(14) -(x(13)-theta1_des)*omega_n^2;  

x15_dot = x(16);
x16_dot = -2*zeta*omega_n*x(16) -(x(15)-theta2_des)*omega_n^2;  

x17_dot = x(18);
x18_dot = -2*zeta_tail*omega_n_tail*x(18) -(x(17)-theta3_des)*omega_n_tail^2;  

x19_dot = x(20);
x20_dot = -2*zeta_tail*omega_n_tail*x(20) -(x(19)-theta4_des)*omega_n_tail^2; 

x21_dot = x(22);
x22_dot = -2*zeta_rotor*omega_n_rotor*x(22) -(x(21)-u(1))*omega_n_rotor^2; 

x23_dot = x(24);
x24_dot = -2*zeta_rotor*omega_n_rotor*x(24) -(x(23)-u(2))*omega_n_rotor^2; 

x25_dot = x(26);
x26_dot = -2*zeta_rotor*omega_n_rotor*x(26) -(x(25)-u(3))*omega_n_rotor^2; 

I_dot_1 = e_PI_1;
I_dot_2 = e_PI_2;

x29_dot = u(10);
x30_dot = u(11);
x31_dot = u(12);

x32_dot = u(13);
x33_dot = u(14);
x34_dot = u(15);
x35_dot = u(16);

%disp("iterazione");

x_dot = [x1_dot;x2_dot;x3_dot;x4_dot;x5_dot;x6_dot;x7_dot;x8_dot;x9_dot;x10_dot;x11_dot;x12_dot;x13_dot;x14_dot;x15_dot;x16_dot;x17_dot;x18_dot;x19_dot;x20_dot;x21_dot;x22_dot;x23_dot;x24_dot;x25_dot;x26_dot;I_dot_1;I_dot_2;x29_dot;x30_dot;x31_dot;x32_dot;x33_dot;x34_dot;x35_dot];

end