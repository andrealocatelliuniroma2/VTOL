%% SIMULAZIONE VTOL 

% Locatelli Andrea 
% A.A 2025-2026

clear all
clc

global p_dot_global q_dot_global r_dot_global
p_dot_global = 0;
q_dot_global = 0;
r_dot_global = 0;


% flag print
paramFlag = 0; % se 1 print del valore dei parametri

% parametri VTOL 
% (roll , pitch , yaw) = (phi , theta , psi)

m = 6.7;      % massa UAV VTOL (Kg)  
g = 9.8;    % costante gravitazionele (m/s^2)

k = 7*(10^-5);      % coeff. thrust rotori
b = 0.005*k; % rapporto tra momento torcente e forza del rotore

%matrice inerzia del corpo (UAV) rispetto al body frame (kg*m^2)
% Ixx = 3;
% Iyy = 0.5 * Ixx;
% Izz = 1.4 * Ixx;

Ixx = 0.237;
Iyy = 0.244;
Izz = 0.468;

I_body = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];     

%inerzie dei tre rotori : wing_dx , wing_sx , tail (kg*m^2)

I_rotor_xx = 0.468;
I_rotor_yy = 0.244;
I_rotor_zz = 0.468;

I_rotor = (10^-3)*[I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];  

I_rotor_w_dx = I_rotor;
I_rotor_w_sx = I_rotor;
I_rotor_tail = I_rotor;


rho = 1.225;    % densità dell'aria (kg/m^3)
ala_y = 0.4;
ala_x = 0.15;
s = ala_x * ala_y;      % superficie alare (m^2) 
v_air = 25;   % velocità relativa alla superficie alare (m/s) lungo x
v_limite = 13.89; % -> 50km/h

C_d = ((m-1)*g)/(rho*s*(v_air)^2); %1.28;  %coeff. di resistenza (drag) aerodinamica lungo asse X
C_y = 0; % trascurabile
% scelto in modo tale che se v_x = 90 km/h (25 m/s) la portanza contrasti
% la gravità
C_l = (m*g)/(rho*s*(v_air)^2); %0.854; %coeff. di portanza (lift) aerodinamica 

% superfici del tricottero
s_body_x = 0.1;
s_body_y = 0.2*0.7;
s_body_z = 0.15*0.7;

% scelto in modo tale che sia raggiunta la velocità limite di 50 km/h
% (13.89 m/s) in caduta libera
C_d_z = (m*g)/(rho*s_body_z*(v_limite)^2); % coeff. di resistenza (drag) aerodinamica lungo asse z
C_d_x = 0.1;
C_d_y = 3;

%distanza (m) tra centro di massa e rotori (per il calcolo del momento dei thrust)
%lungo i vari assi (X_body , Y_body, Z_body)

d_ty = 0;  % assumo il tail rotor posto sull'asse X_body
d_tz = 0;

d_my = (1/2)*ala_y;
d_mz = 0;

d_mx = 0.6;
d_tx = -1.2;

%parametri distanza (m) tra centro di massa e forze aerodinamiche (per il calcolo del momento delle forze aerodinamiche)

l_w_dx_x = 0;%(1/3)*ala_x; % distanza per l'ala dx da centro di massa lungo asse X_body
l_w_dx_y = (1/2)*ala_y; % distanza per l'ala dx da centro di massa lungo asse Y_body
l_w_dx_z = 0; % distanza per l'ala dx da centro di massa lungo asse Z_body

l_w_sx_x = 0;%(1/3)*ala_x; % distanza per l'ala sx da centro di massa lungo asse X_body
l_w_sx_y = -(1/2)*ala_y; % distanza per l'ala sx da centro di massa lungo asse Y_body
l_w_sx_z = 0; % distanza per l'ala sx da centro di massa lungo asse Z_body


% Struttura con tutti i parametri

% Parametri generali
parametri.m = m;
parametri.g = g;
parametri.k = k;

% Inerzia corpo
parametri.Ixx = Ixx;
parametri.Iyy = Iyy;
parametri.Izz = Izz;
parametri.I_body = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];

% Inerzia rotore con Huygens - Steiner
m_rotor = 0.1;
I_rotor_w_dx = I_rotor + m_rotor*[d_mx^2 0 0;0 d_my^2 0;0 0 d_mz^2];
I_rotor_w_sx = I_rotor + m_rotor*[d_mx^2 0 0;0 d_my^2 0;0 0 d_mz^2];
I_rotor_tail = I_rotor + m_rotor*[d_tx^2 0 0;0 d_ty^2 0;0 0 d_tz^2];

parametri.I_rotor_xx = I_rotor_xx;
parametri.I_rotor_yy = I_rotor_yy;
parametri.I_rotor_zz = I_rotor_zz;
parametri.I_rotor = [I_rotor_xx 0 0; 0 I_rotor_yy 0; 0 0 I_rotor_zz];

% uso questi
parametri.I_rotor_w_dx = I_rotor_w_dx;
parametri.I_rotor_w_sx = I_rotor_w_sx;
parametri.I_rotor_tail = I_rotor_tail;

% Parametri aerodinamici
parametri.rho = rho;
parametri.s = s;

parametri.s_body_x = s_body_x;
parametri.s_body_y = s_body_y;
parametri.s_body_z = s_body_z;

parametri.C_d = C_d;
parametri.C_y = C_y;
parametri.C_l = C_l;

parametri.C_d_x = C_d_x;
parametri.C_d_y = C_d_y;
parametri.C_d_z = C_d_z;

parametri.b = b;
parametri.v_air = v_air;

% Parametri ala
parametri.ala_x = ala_x;
parametri.ala_y = ala_y;

% Distanze momento (rotori/forze)
parametri.d_mx = d_mx;
parametri.d_my = d_my;
parametri.d_mz = d_mz;

% Distanze traslazione
parametri.d_tx = d_tx;
parametri.d_ty = d_ty;
parametri.d_tz = d_tz;

% Distanze ala destra
parametri.l_w_dx_x = l_w_dx_x;
parametri.l_w_dx_y = l_w_dx_y;
parametri.l_w_dx_z = l_w_dx_z;

% Distanze ala sinistra
parametri.l_w_sx_x = l_w_sx_x;
parametri.l_w_sx_y = l_w_sx_y;
parametri.l_w_sx_z = l_w_sx_z;
parametri.b = b;

parametri.r_th_w_dx = [d_mx ;  d_my ;  d_mz];
parametri.r_th_w_sx = [d_mx ; -d_my ;  d_mz]; % considero rotore ala dx e sx in posizione simmetrica
parametri.r_th_tail = [d_tx ;  d_ty ;  d_tz];

parametri.l_w_dx = [l_w_dx_x; l_w_dx_y; l_w_dx_z];
parametri.l_w_sx = [l_w_sx_x; l_w_sx_y; l_w_sx_z];

parametri.r_aerodyn_w_dx = parametri.l_w_dx;
parametri.r_aerodyn_w_sx = parametri.l_w_sx;

parametri.switch = 0;

%% SIMULAZIONE

% CONDIZIONI INIZIALI

%x0 = zeros(26,1);            % stato iniziale
x0 = zeros(35,1);

% lo stato 27 è l'errore per fare la parte I del PI_1 del vlo orizzontale
% lo stato 28 è l'errore per fare la parte I del PI_2 del vlo orizzontale

% lo stato 29 è l'integrale dell'errore per fare la parte I del PID per orientamento
% verticale di roll
% lo stato 30 è l'integrale dell'errore per fare la parte I del PID per orientamento
% verticale di pitch
% lo stato 31 è l'integrale dell'errore per fare la parte I del PID per orientamento
% verticale di yaw


voloVerticale = 0; % flag per condizioni iniziali del volo verticale
voloOrizzontale = 0; % flag per condizioni iniziali del volo orizzontale
voloGenerale = 0;
orientamentoVerticale = 1; % flag per condizioni iniziali del volo verticale + orientamento

flagPlot = 1; % grafici + pallina
plotPallina = 0;% pallina
flagPlot3D = 0; % tricottero 3D (volo verticale)
flagPlot3Dorizzontale = 0; % tricottero 3D (volo orizzontale)

if voloVerticale == 1

    x0(4) = 0; % condizione iniziale della velocità lungo X
    x4eq = x0(4);

    % posizione iniziale lungo z
    x0(3) = 0;
    %inclinazione iniziale dei rotori anterioiri (per il volo verticale)
    x0(13)= pi/2;
    x0(15)= pi/2;

    %x0(7)=deg2rad(0.0796); roll compensa Fy

    %inclinazione iniziale del rotore di coda (per il volo verticale)
    %x0(17) = atan2(-parametri.d_tx*parametri.k, parametri.b);
    %x0(19)= -pi/2;

    %inclinazione iniziale del rotore di coda (per il volo verticale senza momento torcente)
    x0(17) = pi/2;
    x0(19)= 0;

elseif voloOrizzontale == 1

    x0(4) = 23; % condizione iniziale della velocità lungo X (25 m/s)
    x4eq = x0(4);

    % posizione iniziale lungo z
    x0(3) = -10;

    % velocità angolare iniziale dei rotori anteriori
    % wind frame (NB:questo blocco di codice è anche in simulazioneVTOL2 e nelle condizioni iniziali del main)
    Va = sqrt((x0(4)^2)+(x0(5)^2)+(x0(6)^2)); % airspeed
    alpha = atan2(x0(6),x0(4)); % angle of attack
    alpha = 0;
    %beta = atan2(x(5),sqrt((x(4)^2)+(x(6)^2))); % sideslip angle
    beta = 0;
    Rwb = matriceRotazioneWingToBodyFrame(alpha,beta);
    F0_x = (1/2)*parametri.rho*parametri.s_body_x*parametri.C_d_x*sign(x0(4))*x0(4)^2 -parametri.rho*parametri.s*(Va^2)*Rwb(1,:)*[-parametri.C_d;parametri.C_y;-parametri.C_l];
    T_i = F0_x/2;
    x0(21)= sqrt(T_i/parametri.k);
    x0(23)= x0(21);

    x0(8) = 0;

    %inclinazione iniziale dei rotori anterioiri (per il volo orizzontale)
    x0(13)= 0;
    x0(15)= 0;

    %inclinazione iniziale del rotore di coda (per il volo orizzontale)
    x0(17) = 0;
    x0(19)= 0;

elseif voloGenerale == 1

    x0(4) = 0; % condizione iniziale della velocità lungo X
    x4eq = x0(4);

    % posizione iniziale lungo z
    x0(3) = 0;

    %x0(7)=deg2rad(0.0796); %roll compensa Fy

    %inclinazione iniziale dei rotori anterioiri
    x0(13)= pi/2;
    x0(15)= pi/2;

    % x0(13)=pi/18;
    % x0(15)=pi/18;

    % x0(13)= 0.552722163282503;
    % x0(15)= 0.552722163282503;

    % x0(21) = 1.024238436881337e+03;
    % x0(23) = 1.024238436881337e+03;
    % x0(25) = 7.421322733312631e+02;

    %inclinazione iniziale del rotore di coda 
    x0(17) = atan2(-parametri.d_tx*parametri.k, parametri.b);
    x0(19)= -pi/2;

elseif orientamentoVerticale == 1

    x0(4) = 0; % condizione iniziale della velocità lungo X
    x4eq = x0(4);

    % posizione iniziale lungo z
    x0(3) = 0;

    %posizione iniziale di roll
    x0(7)=0;

    %posizione iniziale di pitch
    x0(8)=0;

    %posizione iniziale di yaw
    x0(9)=0;

    %inclinazione iniziale dei rotori anterioiri (per il volo verticale)
    x0(13)= pi/2;
    x0(15)= pi/2;

    %inclinazione iniziale del rotore di coda (per il volo verticale)
    x0(17) = pi/2;
    x0(19)= 0;

end

%disp((parametri.d_tx*parametri.k*sin(x0(17))-parametri.b*cos(x0(17)))/(-2*parametri.d_mx*parametri.k));

% SIMULAZIONE CON ODE

tspan = [0 50];              % intervallo di simulazione

options = odeset('RelTol',1e-3,'AbsTol',1e-6);


[t, x] = ode45( @(t, x) simulazioneVTOL2(x,parametri), tspan, x0, options);

%per plot controllo
global U_values
U_values = zeros(length(t),7);
for k = 1:length(t)
    %U_values(k,:) = controlloVTOL_v2(parametri,x(k,:));
    u_1_9 = controlloVTOL_v2(parametri, x(k,:));
    U_values(k,:) = u_1_9(1:7);   % <-- solo le prime 7 componenti
    U_extra(k,:)  = u_1_9(8:9);   % componenti 8 e 9
end

% PLOT


xp = x(:,1);
yp = x(:,2);
zp = -1*x(:,3); % asse z positivo verso il basso

xv = x(:,4);
yv = x(:,5);
zv = -1* x(:,6); % asse z positivo verso il basso

phi = rad2deg(x(:,7));
theta = rad2deg(x(:,8));
psi = rad2deg(x(:,9));

p = x(:,10);
q = x(:,11);
r = x(:,12);

time = linspace(0,tspan(2),size(x,1));

if flagPlot == 1

    % PLOT X1,...,X12

    figure(1)
    set(gcf, 'Position', [100 100 1200 900])

    subplot(4,1,1);
    h1 = plot(time, xp, 'r', time, yp, 'b', time, zp, 'g');
    set(h1, 'LineWidth', 2)
    legend('x_{inertial frame}','y_{inertial frame}','z_{inertial frame}', ...
        'FontSize', 14, 'Interpreter','tex', 'Location','best')
    ylim([-30 30]); grid on
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Posizione [m]', 'FontSize', 14)
    title('Andamento stati (x1,...,x12)','FontSize',16)
    set(gca, 'FontSize', 14)


    subplot(4,1,2);
    h2 = plot(time, xv, 'r', time, yv, 'b', time, zv, 'g');
    set(h2, 'LineWidth', 2)
    legend('vx_{body frame}','vy_{body frame}','vz_{body frame}', ...
        'FontSize', 14, 'Interpreter','tex', 'Location','best')
    ylim([-50 50]); grid on
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Velocità [m/s]', 'FontSize', 14)
    set(gca, 'FontSize', 14)

    subplot(4,1,3);
    h3 = plot(time, phi, 'r', time, theta, 'b', time, psi, 'g');
    set(h3, 'LineWidth', 2)
    legend('\phi (roll,x)','\theta (pitch,y)','\psi (yaw,z)', ...
        'FontSize', 14, 'Interpreter','tex', 'Location','best')
    %ylim([-5 5]);
    grid on;
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Angoli [grad]', 'FontSize', 14)
    set(gca, 'FontSize', 14)

    subplot(4,1,4);
    h4 = plot(time, p, 'r', time, q, 'b', time, r, 'g');
    set(h4, 'LineWidth', 2)
    legend('p','q','r', 'FontSize', 14, 'Interpreter','tex', 'Location','best')
    %ylim([-5 5]);
    grid on;
    xlabel('Time [s]', 'FontSize', 14)
    ylabel('Vel. angolari [rad/s]', 'FontSize', 14)
    set(gca, 'FontSize', 14)

    if voloVerticale == 1
        % grafici volo verticale

        figure(2)
        set(gcf, 'Position', [100 100 1200 900])

        % --- vz ---
        subplot(3,1,2);
        h3=plot(time, zv, 'b', 'LineWidth', 2); hold on;
        h4=yline(0,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
        grid on; ylim([-20 20])
        xlabel('Time [s]', 'FontSize', 14)
        ylabel('v_z [m/s]', 'FontSize', 14)
        title('Velocità lungo z','FontSize',16)
        set(gca, 'FontSize', 14)
        legend([h3 h4], {'v_z','vz_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')


        % --- z (quota) ---
        subplot(3,1,1);
        h5=plot(time, zp, 'g', 'LineWidth', 2); hold on;
        h6=yline(10,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
        grid on; ylim([-16 16])
        xlabel('Time [s]', 'FontSize', 14)
        ylabel('Quota z [m]', 'FontSize', 14)
        title('Posizione lungo z','FontSize',16)
        set(gca, 'FontSize', 14)
        legend([h5 h6], {'z','z_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')

        % --- y (posizione lungo y) ---
        subplot(3,1,3);
        h5=plot(time, yp, 'r', 'LineWidth', 2); hold on;
        h6=yline(0,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
        grid on; ylim([-10 10])
        xlabel('Time [s]', 'FontSize', 14)
        ylabel('Posizione y [m]', 'FontSize', 14)
        title('Posizione lungo y','FontSize',16)
        set(gca, 'FontSize', 14)
        legend([h5 h6], {'y','y_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')

        % thrust genearato dai rotori

        omega_1 = x(:,21);
        omega_2 = x(:,23);
        omega_3 = x(:,25);

        figure(3)
        set(gcf, 'Position', [100 100 1200 900])

        subplot(3,1,1);
        h1 = plot(time, parametri.k*omega_1.^2, 'r','LineWidth',2);
        legend('Thrust_{1}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        set(gca,'FontSize',14)
        title('Thrust generato dai rotori','FontSize',16)

        subplot(3,1,2);
        h2 = plot(time, parametri.k*omega_2.^2, 'r','LineWidth',2);
        legend('Thrust_{2}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        set(gca,'FontSize',14)

        subplot(3,1,3);
        h3 = plot(time, parametri.k*omega_3.^2, 'r','LineWidth',2);
        legend('Thrust_{3}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        set(gca,'FontSize',14)

        figure(4)
        set(gcf, 'Position', [100 100 1200 900])

        theta1 = rad2deg(x(:,13));
        theta2 = rad2deg(x(:,15));
        theta3 = rad2deg(x(:,17));
        theta4 = rad2deg(x(:,19));

        subplot(4,1,1);
        h1 = plot(time, theta1, 'r','LineWidth',2);
        legend('\theta_1','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        title('Andamento angoli di tilt dei rotori','FontSize',16)
        set(gca,'FontSize',14)

        subplot(4,1,2);
        h2 = plot(time, theta2, 'b','LineWidth',2);
        legend('\theta_2','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        set(gca,'FontSize',14)

        subplot(4,1,3);
        h3 = plot(time, theta3, 'g','LineWidth',2);
        legend('\theta_3','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        title('Andamento angoli di tilt dei rotori','FontSize',16)
        set(gca,'FontSize',14)

        subplot(4,1,4);
        h4 = plot(time, theta4, 'g','LineWidth',2);
        legend('\theta_4','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        set(gca,'FontSize',14)

    end

    if voloOrizzontale == 1
        % grafici volo verticale

        figure(2)
        set(gcf, 'Position', [100 100 1200 900])

        % --- vz ---
        subplot(3,1,1);
        h3=plot(time, xv, 'r', 'LineWidth', 2); hold on;
        h4=yline(25,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
        grid on; ylim([-10 40])
        xlabel('Time [s]', 'FontSize', 14)
        ylabel('v_z [m/s]', 'FontSize', 14)
        title('Velocità lungo x','FontSize',16)
        set(gca, 'FontSize', 14)
        legend([h3 h4], {'v_x','vx_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')

        % --- vz ---
        subplot(3,1,3);
        h5=plot(time, zv, 'b', 'LineWidth', 2); hold on;
        h6=yline(0,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
        grid on; ylim([-20 20])
        xlabel('Time [s]', 'FontSize', 14)
        ylabel('v_z [m/s]', 'FontSize', 14)
        title('Velocità lungo z','FontSize',16)
        set(gca, 'FontSize', 14)
        legend([h3 h4], {'v_z','vz_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')


        % --- z (quota) ---
        subplot(3,1,2);
        h7=plot(time, zp, 'g', 'LineWidth', 2); hold on;
        h8=yline(10,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
        grid on; ylim([-50 20])
        xlabel('Time [s]', 'FontSize', 14)
        ylabel('Quota z [m]', 'FontSize', 14)
        title('Posizione lungo z','FontSize',16)
        set(gca, 'FontSize', 14)
        legend([h5 h6], {'z','z_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')


        % thrust genearato dai rotori

        omega_1 = x(:,21);
        omega_2 = x(:,23);
        omega_3 = x(:,25);

        figure(3)
        set(gcf, 'Position', [100 100 1200 900])

        subplot(3,1,1);
        h1 = plot(time, parametri.k*omega_1.^2, 'r','LineWidth',2);
        legend('Thrust_{1}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        ylim([0 100])
        set(gca,'FontSize',14)
        title('Thrust generato dai rotori','FontSize',16)

        subplot(3,1,2);
        h2 = plot(time, parametri.k*omega_2.^2, 'r','LineWidth',2);
        legend('Thrust_{2}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        ylim([0 100])
        set(gca,'FontSize',14)

        subplot(3,1,3);
        h3 = plot(time, parametri.k*omega_3.^2, 'r','LineWidth',2);
        legend('Thrust_{3}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        ylim([0 100])
        set(gca,'FontSize',14)

        figure(4)
        set(gcf, 'Position', [100 100 1200 900])

        theta1 = rad2deg(x(:,13));
        theta2 = rad2deg(x(:,15));
        theta3 = rad2deg(x(:,17));
        theta4 = rad2deg(x(:,19));

        subplot(4,1,1);
        h1 = plot(time, theta1, 'r','LineWidth',2);
        legend('\theta_1','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        title('Andamento angoli di tilt dei rotori','FontSize',16)
        set(gca,'FontSize',14)

        subplot(4,1,2);
        h2 = plot(time, theta2, 'b','LineWidth',2);
        legend('\theta_2','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        set(gca,'FontSize',14)

        subplot(4,1,3);
        h3 = plot(time, theta3, 'g','LineWidth',2);
        legend('\theta_3','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        title('Andamento angoli di tilt dei rotori','FontSize',16)
        set(gca,'FontSize',14)

        subplot(4,1,4);
        h4 = plot(time, theta4, 'g','LineWidth',2);
        legend('\theta_4','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        set(gca,'FontSize',14)

    end

    if voloGenerale == 1
        % grafici volo verticale

        figure(2)
        set(gcf, 'Position', [100 100 1200 900])

        % --- vz ---
        subplot(3,1,1);
        h3=plot(time, xv, 'r', 'LineWidth', 2); hold on;
        h4=yline(25,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
        grid on; ylim([-10 40])
        xlabel('Time [s]', 'FontSize', 14)
        ylabel('v_x [m/s]', 'FontSize', 14)
        title('Velocità lungo x','FontSize',16)
        set(gca, 'FontSize', 14)
        legend([h3 h4], {'v_x','vx_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')

        % --- vz ---
        subplot(3,1,3);
        h5=plot(time, zv, 'b', 'LineWidth', 2); hold on;
        h6=yline(0,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
        grid on; ylim([-20 20])
        xlabel('Time [s]', 'FontSize', 14)
        ylabel('v_z [m/s]', 'FontSize', 14)
        title('Velocità lungo z','FontSize',16)
        set(gca, 'FontSize', 14)
        legend([h5 h6], {'v_z','vz_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')


        % --- z (quota) ---
        subplot(3,1,2);
        h7=plot(time, zp, 'g', 'LineWidth', 2); hold on;
        h8=yline(10,'--k','LabelHorizontalAlignment','left','FontSize',12,'LineWidth', 2);
        grid on; ylim([-16 16])
        xlabel('Time [s]', 'FontSize', 14)
        ylabel('Quota z [m]', 'FontSize', 14)
        title('Posizione lungo z','FontSize',16)
        set(gca, 'FontSize', 14)
        legend([h7 h8], {'z','z_{des}'}, 'Interpreter','tex','FontSize',12,'Location','best')


        % thrust genearato dai rotori

        omega_1 = x(:,21);
        omega_2 = x(:,23);
        omega_3 = x(:,25);

        figure(3)
        set(gcf, 'Position', [100 100 1200 900])

        subplot(3,1,1);
        h1 = plot(time, parametri.k*omega_1.^2, 'r','LineWidth',2);
        legend('Thrust_{1}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        ylim([0 100])
        set(gca,'FontSize',14)
        title('Thrust generato dai rotori','FontSize',16)

        subplot(3,1,2);
        h2 = plot(time, parametri.k*omega_2.^2, 'r','LineWidth',2);
        legend('Thrust_{2}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        ylim([0 100])
        set(gca,'FontSize',14)

        subplot(3,1,3);
        h3 = plot(time, parametri.k*omega_3.^2, 'r','LineWidth',2);
        legend('Thrust_{3}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        ylim([0 100])
        set(gca,'FontSize',14)

        figure(4)
        set(gcf, 'Position', [100 100 1200 900])

        theta1 = rad2deg(x(:,13));
        theta2 = rad2deg(x(:,15));
        theta3 = rad2deg(x(:,17));
        theta4 = rad2deg(x(:,19));

        subplot(4,1,1);
        h1 = plot(time, theta1, 'r','LineWidth',2);
        legend('\theta_1','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        title('Andamento angoli di tilt dei rotori','FontSize',16)
        set(gca,'FontSize',14)

        subplot(4,1,2);
        h2 = plot(time, theta2, 'b','LineWidth',2);
        legend('\theta_2','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        set(gca,'FontSize',14)

        subplot(4,1,3);
        h3 = plot(time, theta3, 'g','LineWidth',2);
        legend('\theta_3','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        title('Andamento angoli di tilt dei rotori','FontSize',16)
        set(gca,'FontSize',14)

        subplot(4,1,4);
        h4 = plot(time, theta4, 'g','LineWidth',2);
        legend('\theta_4','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        set(gca,'FontSize',14)

    end

    if orientamentoVerticale ==1

        % thrust genearato dai rotori

        omega_1 = x(:,21);
        omega_2 = x(:,23);
        omega_3 = x(:,25);

        figure(3)
        set(gcf, 'Position', [100 100 1200 900])

        subplot(3,1,1);
        h1 = plot(time, parametri.k*omega_1.^2, 'r','LineWidth',2);
        legend('Thrust_{1}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        set(gca,'FontSize',14)
        title('Thrust generato dai rotori','FontSize',16)

        subplot(3,1,2);
        h2 = plot(time, parametri.k*omega_2.^2, 'r','LineWidth',2);
        legend('Thrust_{2}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        set(gca,'FontSize',14)

        subplot(3,1,3);
        h3 = plot(time, parametri.k*omega_3.^2, 'r','LineWidth',2);
        legend('Thrust_{3}','FontSize',14,'Location','best')
        grid on
        ylabel('[N]','FontSize',14)
        set(gca,'FontSize',14)

        figure(4)
        set(gcf, 'Position', [100 100 1200 900])

        theta1 = rad2deg(x(:,13));
        theta2 = rad2deg(x(:,15));
        theta3 = rad2deg(x(:,17));
        theta4 = rad2deg(x(:,19));

        subplot(4,1,1);
        h1 = plot(time, theta1, 'r','LineWidth',2);
        legend('\theta_1','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        title('Andamento angoli di tilt dei rotori','FontSize',16)
        set(gca,'FontSize',14)

        subplot(4,1,2);
        h2 = plot(time, theta2, 'b','LineWidth',2);
        legend('\theta_2','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        set(gca,'FontSize',14)

        subplot(4,1,3);
        h3 = plot(time, theta3, 'g','LineWidth',2);
        legend('\theta_3','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        title('Andamento angoli di tilt dei rotori','FontSize',16)
        set(gca,'FontSize',14)

        subplot(4,1,4);
        h4 = plot(time, theta4, 'g','LineWidth',2);
        legend('\theta_4','FontSize',14,'Location','best')
        grid on
        ylabel('[grad]','FontSize',14)
        set(gca,'FontSize',14)

    end

    if plotPallina == 1

        figure(5);
        set(gcf, 'Position', [100 100 1200 900])
        axis equal
        hold on;

        % Traiettoria completa in 3D
        plot3(xp, yp, zp, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.0);

        % Punto mobile (tricottero)
        hP = plot3(xp(1), yp(1), zp(1), 'ro', ...
            'MarkerSize', 15, 'MarkerFaceColor', 'r');

        grid on;
        axis equal;
        xlabel('X [m]');
        ylabel('Y [m]');
        zlabel('Z [m]');
        title('Animazione 3D del tricottero');

        % Vista 3D
        view(3);


        for k = 1:5:length(t)
            set(hP, 'XData', xp(k), ...
                'YData', yp(k), ...
                'ZData', zp(k));
            drawnow;
        end

    end

end


if flagPlot3D == 1

    TricopterPlot(t,x);

end

if flagPlot3Dorizzontale == 1

    TricopterPlot2(t,x);

end


%% test orientamento angoli

% phi = 0;
% theta = pi/4;
% psi = 0;
% 
% e_x = [1;0;0];
% e_y = [0;1;0];
% e_z = [0;0;1];
% 
% R = matriceRotazione(phi,theta,psi);
% disp("Matrice di rotazione R(b->g):")
% disp(R);
% 
% disp("Direzione Xb (muso):")
% disp(R*e_x);



%% test direzione rotore di coda

% %inclinazione iniziale dei rotori anterioiri (per il volo verticale)
% x0(13)= pi/2;
% x0(15)= pi/2;
% %inclinazione iniziale del rotore di coda (per il volo verticale)
% x0(17) = atan2(-parametri.d_tx*parametri.k, parametri.b);
% x0(19)= -pi/2;
% 
% e_x = [1;0;0];
% 
% s3 = sin(x0(17));
% c3 = cos(x0(17));
% s4 = sin(x0(19));
% c4 = cos(x0(19));
% R_tail = [c4*c3 -s4 c4*s3;s4*c3 c4 s4*s3;-s3 0 c3];
% disp(R_tail);
% 
% disp("Direzione rotore di coda:")
% disp(R_tail*e_x);


%% test wing frame
% syms Cd Cy Cl
% 
% % volo verticale
% alpha = pi/2;
% beta = 0;
% coeffAero = [-Cd;Cy;-Cl];
% Rwb = round(matriceRotazioneWingToBodyFrame(alpha,beta),3);
% disp(Rwb*coeffAero)

%% plot controllo
% 
% figure(7)
% set(gcf,'Position',[100 100 1000 800]) % finestra più grande
% for i = 1:3
%     subplot(3,1,i)
%     h = plot(t, parametri.k*U_values(:,i).^2, 'LineWidth', 2);
%     ylabel(sprintf('u_%d [N]', i), 'FontSize',14)
%     grid on
%     set(gca,'FontSize',14)
%     if i==1
%         title('Andamento del thrust desiderato dei rotori','FontSize',16) % titolo generale sul primo subplot
%     end
%     if i==3
%         xlabel('Tempo [s]','FontSize',14)
%     end
% end
% 
% figure(8)
% set(gcf,'Position',[100 100 1000 900]) % finestra più grande
% for i = 4:7
%     subplot(4,1,i-3)  
%     h = plot(t, rad2deg(U_values(:,i)), 'LineWidth', 2);
%     ylabel(sprintf('u_%d [rad]', i), 'FontSize',14)
%     grid on
%     set(gca,'FontSize',14)
%     if i==4
%         title('Andamento delle inclinazioni desiderate dei rotori','FontSize',16) % titolo generale sul primo subplot
%     end
%     if i==7
%         xlabel('Tempo [s]','FontSize',14)
%     end
% end

%%
% figure(10)
% set(gcf, 'Position', [100 100 1200 900])
% 
% % Estrazione thrust reali
% T1_real = parametri.k * omega_1.^2;
% T2_real = parametri.k * omega_2.^2;
% T3_real = parametri.k * omega_3.^2;
% 
% % Estrazione thrust desiderati
% T1_des = parametri.k * U_values(:,1).^2;
% T2_des = parametri.k * U_values(:,2).^2;
% T3_des = parametri.k * U_values(:,3).^2;
% 
% % ------ SUBPLOT 1: rotore 1 ------
% subplot(3,1,1)
% plot(time, T1_real, 'r', 'LineWidth', 2); hold on
% plot(time, T1_des,  'b--', 'LineWidth', 2);
% ylabel('[N]', 'FontSize', 14)
% legend('Thrust_{1} real','Thrust_{1} desired','FontSize',14,'Location','best')
% grid on
% set(gca, 'FontSize', 14)
% title('Thrust reale vs desiderato dei rotori','FontSize',16)
% 
% % ------ SUBPLOT 2: rotore 2 ------
% subplot(3,1,2)
% plot(time, T2_real, 'r', 'LineWidth', 2); hold on
% plot(time, T2_des,  'b--', 'LineWidth', 2);
% ylabel('[N]', 'FontSize', 14)
% legend('Thrust_{2} real','Thrust_{2} desired','FontSize',14,'Location','best')
% grid on
% set(gca, 'FontSize', 14)
% 
% % ------ SUBPLOT 3: rotore 3 (coda) ------
% subplot(3,1,3)
% plot(time, T3_real, 'r', 'LineWidth', 2); hold on
% plot(time, T3_des,  'b--', 'LineWidth', 2);
% ylabel('[N]', 'FontSize', 14)
% legend('Thrust_{3} real','Thrust_{3} desired','FontSize',14,'Location','best')
% grid on
% set(gca, 'FontSize', 14)
% xlabel('Tempo [s]', 'FontSize', 14)
% 
% %%
% 
% figure(20)
% set(gcf,'Position',[100 100 1200 900])
% 
% % Angoli REALI
% theta1_real = rad2deg(x(:,13));
% theta2_real = rad2deg(x(:,15));
% theta3_real = rad2deg(x(:,17));
% theta4_real = rad2deg(x(:,19));
% 
% % Angoli DESIDERATI (U_values = comandi)
% theta1_des = rad2deg(U_values(:,4));
% theta2_des = rad2deg(U_values(:,5));
% theta3_des = rad2deg(U_values(:,6));
% theta4_des = rad2deg(U_values(:,7));
% 
% % ---------- SUBPLOT 1 ----------
% subplot(4,1,1)
% plot(time, theta1_real, 'r', 'LineWidth', 2); hold on
% plot(time, theta1_des,  'b--', 'LineWidth', 2);
% ylabel('[deg]', 'FontSize', 14)
% legend('θ_1 real','θ_1 desired', 'FontSize',14, 'Location','best')
% grid on
% set(gca,'FontSize',14)
% title('Andamento angoli di tilt dei rotori (reali vs desiderati)','FontSize',16)
% 
% % ---------- SUBPLOT 2 ----------
% subplot(4,1,2)
% plot(time, theta2_real, 'r', 'LineWidth', 2); hold on
% plot(time, theta2_des,  'b--', 'LineWidth', 2);
% ylabel('[deg]', 'FontSize', 14)
% legend('θ_2 real','θ_2 desired', 'FontSize',14, 'Location','best')
% grid on
% set(gca,'FontSize',14)
% 
% % ---------- SUBPLOT 3 ----------
% subplot(4,1,3)
% plot(time, theta3_real, 'r', 'LineWidth', 2); hold on
% plot(time, theta3_des,  'b--', 'LineWidth', 2);
% ylabel('[deg]', 'FontSize', 14)
% legend('θ_3 real','θ_3 desired', 'FontSize',14, 'Location','best')
% grid on
% set(gca,'FontSize',14)
% 
% % ---------- SUBPLOT 4 ----------
% subplot(4,1,4)
% plot(time, theta4_real, 'r', 'LineWidth', 2); hold on
% plot(time, theta4_des,  'b--', 'LineWidth', 2);
% ylabel('[deg]', 'FontSize', 14)
% legend('θ_4 real','θ_4 desired', 'FontSize',14, 'Location','best')
% grid on
% set(gca,'FontSize',14)
% xlabel('Tempo [s]', 'FontSize', 14)
%%
% figure(21);
% set(gcf,'Position',[100 100 1200 900])
% subplot(2,1,1);
% plot(t, U_extra(:,1));
% title('u_8');
% grid on
% subplot(2,1,2);
% plot(t, U_extra(:,2));
% title('u_9');
% grid on
% 
% figure(22);
% set(gcf,'Position',[100 100 1200 900])
% e1 = x(:,27);
% e2 = x(:,28);
% subplot(2,1,1);
% plot(t, e1);
% grid on
% subplot(2,1,2);
% plot(t, e2);
% grid on