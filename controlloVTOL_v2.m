function u = controlloVTOL_v2(params, x)

% Preallocazione
u = zeros(16,1);

test_id = 14;
% TEST

% -1 : Debug (tutto simbolico)

% 0 : Nessun controllo (solo gravità)

% 1 : compensazione attrito lungo X 

% 2 : controllo velocità lungo X e posizione e velocità lungo Z

% 3 : volo verticale (senza momento torcente)  
% NB: per il test 3
% dmx = -(1/2)*dtx (serve a compensare momento di pitch)
% usare M_thrust_noTorc (tolgo momento torcente)
% mettere il rotore di coda inclinato verticalmente (x0(17)= pi/2; x0(19)=0)

% 4 : volo verticale (con momento torcente)  

% 5 : volo orizzontale (rotore di coda spento)

% 6 : transizione da volo verticale a volo orizzontale (non funzionante)

% 7 : transizione da volo verticale a volo orizzontale (funzionante ma senza dinamica rotori e tilting)

% 12 : orientamento 

switch test_id

    case -1
        %debug
        syms u1 u2 u3 u4 u5 u6 u7
        u = [u1;u2;u3;u4;u5;u6;u7];
        
        % variante in assenza di rotore di coda 
        u = [u1;u2;0;u4;u5;0;0];

    case 0
        % Nessun controllo (solo gravità)
        u = zeros(7,1);

    case 1

        u = zeros(7,1);

        x4eq = 25;

        D=(params.rho*params.s*params.C_d*(x4eq)^2);
  
        omega1_2 = (0.5)*(D/params.k);
        omega2_2 = (0.5)*(D/params.k);
        omega3_2 = (0.0)*(D/params.k);


        u(1) = sqrt(omega1_2);
        u(2) = sqrt(omega2_2);
        u(3) = sqrt(omega3_2);  



    case 2

        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % controllo 

        u = zeros(7,1);
        
        vx_des = 25;
        kp_x = 5;
        F_x_des = params.rho*params.s*params.C_d*sign(x(4))*x(4)^2;
        F_x_des = F_x_des +kp_x*(vx_des-x(4));

        kp_z = -3;
        kd_z = -5;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;      
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD



        % theta_bar = atan2(F_z_des,F_x_des);
        % 
        % u(4) = theta_bar;
        % u(5) =u(4);
        % u(1) = sqrt((F_z_des/(2*sin(u(4))))/params.k);
        % u(2)=u(1);

        T_tot = sqrt(F_x_des^2 + F_z_des^2);
        theta_bar = atan2(F_z_des,F_x_des);

        % Assegna ai due rotori anteriori
        T_i = T_tot/2;

        u(1) = sqrt(T_i/params.k);   % omega1
        u(2) = u(1);                 % omega2
        u(4) = theta_bar;             % tilt rotore 1
        u(5) = theta_bar;             % tilt rotore 2

    
    case 3

        % NB: per questo test annullare momento torcente
        % e dmx = -(1/2)*dtx  (serve a compensare momento di pitch)

        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        %vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % controllo

        u = zeros(7,1);

        % durante la fase di volo verticale, i rotori generano thrust verso
        % l'alto

        u(4)=pi/2;
        u(5)=pi/2;
        u(6)=pi/2;
        u(7)=0;

        kp_z = -5;
        kd_z = -10;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

        T_tot = F_z_des;
        T_i = T_tot/3;

        u(1) = sqrt(T_i/params.k);   % omega1
        u(2) = u(1);                 % omega2
        u(3) = u(1);                 % omega3

    case 4

        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame  

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        %vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % controllo

        u = zeros(9,1);

        kp_z = -5;
        kd_z = -10;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

        if F_z_des < 0
            test = 1;
        end

        theta3= atan2(((-params.d_tx*params.k)/params.b),1);
        omega3_2 = (params.d_mx*F_z_des)/(params.d_mx*params.k*sin(theta3)+params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));
        omega_2= (F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k);
        
        if omega3_2 < 0
            test = 1;
        end

        
        %check
        % term1 = -params.d_tx*params.k*omega3_2*cos(theta3);
        % term2 = -params.b*omega3_2*sin(theta3);
        % term = term1+term2;
        % term3 = 2*params.d_mx*params.k*omega_2;
        % term4 = params.d_tx*params.k*sin(theta3)*omega3_2;
        % term5 = -params.b*cos(theta3)*omega3_2;
        % term = term3+term4+term5;
        % term6 = 2*params.k*omega_2;
        % term7 = F_z_des-params.k*sin(theta3)*omega3_2;
        % term = term6 -term7;

        u(1)=sqrt(omega_2);
        u(2)=u(1);
        u(3)=sqrt(omega3_2);
        u(4)=pi/2;
        u(5)=pi/2;
        u(6)=theta3;
        u(7)=-pi/2;

    case 5 % Volo orizzontale

        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        vx_global = V_global(1);

        % wind frame (NB:questo blocco di codice è anche in simulazioneVTOL2 e nelle condizioni iniziali del main)
        Va = sqrt((x(4)^2)+(x(5)^2)+(x(6)^2)); % airspeed
        %alpha = atan2(x(6),x(4)); % angle of attack
        %beta = atan2(x(5),sqrt((x(4)^2)+(x(6)^2))); % sideslip angle
        alpha = 0;
        beta = 0;
        Rwb = matriceRotazioneWingToBodyFrame(alpha,beta);

        % controllo

        u = zeros(9,1);

        vx_des = 25;

        persistent switching_control_PI
        if isempty(switching_control_PI)
            switching_control_PI = 0;
        end
       

        kp_x = 5;

        % if abs(x(4)-vx_des)<1e-5 || switching_control_PI==1
        %     ki_x = 0.1;
        %     switching_control_PI = 1;
        %     I = ki_x*x(28);
        %     u(9) = vx_des-x(4);
        %     u(8) = 0;
        % else
        %     ki_x = 10;
        %     switching_control_PI = 0;
        %     I = ki_x*x(27);
        %     u(8) = vx_des-x(4);
        %     u(9) = 0;
        % end

        ki_x = 10;
        I = ki_x*x(27);
        u(8) = vx_des-x(4);      

        F_x_des = (1/2)*params.rho*params.s_body_x*params.C_d_x*sign(x(4))*x(4)^2 -params.rho*params.s*(Va^2)*Rwb(1,:)*[-params.C_d;params.C_y;-params.C_l];
        F_x_des = F_x_des +kp_x*(vx_des-vx_global)+I;

        % Assegna ai due rotori anteriori
        T_i = F_x_des/2;

        u(1) = sqrt(T_i/params.k);   % omega1
        u(2) = u(1);                 % omega2
        u(3) = 0;                    % omega3 -> rotore di coda spento
        u(4) = 0;             % tilt rotore 1
        u(5) = 0;             % tilt rotore 2
        u(6) = 0;
        u(7) = 0;

    case 6

        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        %vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % inizializzazione controllo
        u = zeros(7,1);

        % COMPONENTE ORIZZONTALE

        % wind frame (NB:questo blocco di codice è anche in simulazioneVTOL2 e nelle condizioni iniziali del main)
        Va = sqrt((x(4)^2)+(x(5)^2)+(x(6)^2)); % airspeed
        %alpha = atan2(x(6),x(4)); % angle of attack
        %beta = atan2(x(5),sqrt((x(4)^2)+(x(6)^2))); % sideslip angle
        alpha = 0;
        beta = 0;
        Rwb = matriceRotazioneWingToBodyFrame(alpha,beta);

        vx_des = 25;
        kp_x = 5;
        F_x_des = (1/2)*params.rho*params.s_body_x*params.C_d_x*sign(x(4))*x(4)^2 -params.rho*params.s*(Va^2)*Rwb(1,:)*[-params.C_d;params.C_y;-params.C_l];
        F_x_des = F_x_des +kp_x*(vx_des-x(4));


        % COMPONENTE VERTICALE

        kp_z = -5;
        kd_z = -10;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

        if F_z_des < 0
            test = 1;
        end


        % CONTROLLO

        theta3 = atan2(((-params.d_tx*params.k)/params.b),1);
        omega3_2 = (params.d_mx*F_z_des)/(params.d_mx*params.k*sin(theta3)+params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));
        if omega3_2 < 0
            test = 1;
        end
      
        if ~isreal(F_z_des-(omega3_2*params.k*sin(theta3)))
            test = 1;
        end

        if ~isreal(F_x_des)
            test = 1;
        end

        %theta12 = atan2((F_z_des-(omega3_2*params.k*sin(theta3))),F_x_des);
        theta12 = max(0,x(13)-0.01);

        %theta12 = pi/4;
        % if theta12 > pi/4
        %     omega_2_new = (F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k*sin(theta12));
        % else
        %     omega_2_new = (F_x_des)/(2*params.k*cos(theta));
        % end
        
        %omega_2_new = sqrt((((F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k))^2)+((F_x_des/(2*params.k))^2));
        
        if omega_2_new < 0
            test = 1;
        end


        %check
        % term1 = -params.d_tx*params.k*omega3_2*cos(theta3);
        % term2 = -params.b*omega3_2*sin(theta3);
        % term = term1+term2;
        % term3 = 2*params.d_mx*params.k*omega_2;
        % term4 = params.d_tx*params.k*sin(theta3)*omega3_2;
        % term5 = -params.b*cos(theta3)*omega3_2;
        % term = term3+term4+term5;
        % term6 = 2*params.k*omega_2;
        % term7 = F_z_des-params.k*sin(theta3)*omega3_2;
        % term = term6 -term7;

        % u(1)=sqrt(omega_2);
        % u(2)=u(1);
        % u(3)=sqrt(omega3_2);
        % u(4)=pi/2;
        % u(5)=pi/2;
        % u(6)=theta3;
        % u(7)=-pi/2;

        u(1)=sqrt(omega_2_new);
        u(2)=u(1);
        u(3)=sqrt(omega3_2);
        u(4)=theta12;
        u(5)=theta12;
        u(6)=theta3;
        u(7)=-pi/2;

    case 7  % FUNZIONA CON input_thrust = [params.k*u(1)^2;params.k*u(2)^2;params.k*u(3)^2;u(4);u(5);u(6);u(7)];

        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        %vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % inizializzazione controllo
        u = zeros(7,1);

        % COMPONENTE ORIZZONTALE

        % wind frame (NB:questo blocco di codice è anche in simulazioneVTOL2 e nelle condizioni iniziali del main)
        Va = sqrt((x(4)^2)+(x(5)^2)+(x(6)^2)); % airspeed
        %alpha = atan2(x(6),x(4)); % angle of attack
        %beta = atan2(x(5),sqrt((x(4)^2)+(x(6)^2))); % sideslip angle
        alpha = 0;
        beta = 0;
        Rwb = matriceRotazioneWingToBodyFrame(alpha,beta);

        vx_des = 25;
        kp_x = 5;
        F_x_des = (1/2)*params.rho*params.s_body_x*params.C_d_x*sign(x(4))*x(4)^2 -params.rho*params.s*(Va^2)*Rwb(1,:)*[-params.C_d;params.C_y;-params.C_l];
        F_x_des = F_x_des +kp_x*(vx_des-x(4));


        % COMPONENTE VERTICALE

        kp_z = -5;
        kd_z = -10;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

        if F_z_des < 0
            test = 1;
        end

        theta3_des = atan2(((-params.d_tx*params.k)/params.b),1);
        theta3 = x(17);

        omega2sin = F_z_des/(2*params.k -params.k*sin(theta3)*((2*params.d_mx*params.k)/(-params.b*cos(theta3)+params.d_tx*params.k*sin(theta3))));
        omega2cos = F_x_des/(2*params.k);

        theta12_des = atan2(omega2sin,omega2cos);
        theta12 = x(13);

        omega2_des = sqrt((omega2sin^2)+(omega2cos^2));
        omega2 =x(21)^2; 

        if omega2_des < 0
            test = 1;
        end

        omega3_2_des = (-2*params.d_mx*params.k*omega2_des*sin(theta12_des))/(-params.b*cos(theta3)+params.d_tx*params.k*sin(theta3));
        %omega3_2_des = (-2*params.d_mx*params.k*omega2*sin(theta12))/(-params.b*cos(theta3)+params.d_tx*params.k*sin(theta3));

        if omega3_2_des < 0
            test = 1;
        end

        %check
        % term1 = -params.d_tx*params.k*omega3_2*cos(theta3);
        % term2 = -params.b*omega3_2*sin(theta3);
        % term = term1+term2;
        % term3 = 2*params.d_mx*params.k*omega2;
        % term4 = params.d_tx*params.k*sin(theta3)*omega3_2;
        % term5 = -params.b*cos(theta3)*omega3_2;
        % term = term3+term4+term5;
        % term6 = 2*params.k*omega2;
        % term7 = F_z_des-params.k*sin(theta3)*omega3_2;
        % term = term6 -term7;


        % CONTROLLO

        u(1)=sqrt(omega2_des);
        u(2)=u(1);
        u(3)=sqrt(omega3_2_des);
        u(4)=theta12_des;
        u(5)=theta12_des;
        u(6)=theta3_des;
        u(7)=-pi/2;


    case 8  
        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        %vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % controllo

        u = zeros(7,1);

        kp_z = -5;
        kd_z = -10;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

        if F_z_des < 0
            test = 1;
        end

        if 0 % inizio la transizione

            


        else % volo verticale

            theta3= atan2(((-params.d_tx*params.k)/params.b),1);
            %theta12 = pi/6;
            theta12 = pi/2;
            omega3_2 = (params.d_mx*F_z_des)/(params.d_mx*params.k*sin(theta3)+params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));
            omega_2= (F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k*sin(theta12));
            

        end
       
        if omega3_2 < 0
            test = 1;
        end

        if abs(z_des-x(3))<0.01 || x(13)~=pi/2

            % COMPONENTE ORIZZONTALE

            % wind frame (NB:questo blocco di codice è anche in simulazioneVTOL2 e nelle condizioni iniziali del main)
            Va = sqrt((x(4)^2)+(x(5)^2)+(x(6)^2)); % airspeed
            %alpha = atan2(x(6),x(4)); % angle of attack
            %beta = atan2(x(5),sqrt((x(4)^2)+(x(6)^2))); % sideslip angle
            alpha = 0;
            beta = 0;
            Rwb = matriceRotazioneWingToBodyFrame(alpha,beta);

            vx_des = 25;
            kp_x = 10;
            F_x_des = (1/2)*params.rho*params.s_body_x*params.C_d_x*sign(x(4))*x(4)^2 -params.rho*params.s*(Va^2)*Rwb(1,:)*[-params.C_d;params.C_y;-params.C_l];
            F_x_des = F_x_des +kp_x*(vx_des-x(4));

            theta3= atan2(((-params.d_tx*params.k)/params.b),1);

            eps = 10^-3;
            theta12 = max(x(13)-eps,0); % 0.035
            %omega_2 = 0;%F_x_des/(2*params.k*cos(theta12));
            %omega3_2 =0;%(-2*params.d_mx*params.k*omega_2*sin(theta12)/(params.d_tx*params.k*sin(theta3)-params.b*cos(theta3)));

            F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
            F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

            %omega3_2 = (params.d_mx*F_z_des)/(params.d_mx*params.k*sin(theta3)+params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));
            %omega_2= (F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k*sin(theta12));


            if abs(sin(theta12)) < 0

                % test 1
                % theta12 = x(13);
                % 
                % omega_2 = F_x_des/(2*params.k*cos(theta12));
                % omega3_2 = (2*params.d_mx*params.k*omega_2*sin(theta12))/(params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));

                % test 2
                % theta3= atan2(((-params.d_tx*params.k)/params.b),1);
                % 
                % omega2sin = F_z_des/(2*params.k -params.k*sin(theta3)*((2*params.d_mx*params.k)/(-params.b*cos(theta3)+params.d_tx*params.k*sin(theta3))));
                % omega2cos = F_x_des/(2*params.k);
                % 
                % omega3_2 = (-2*params.d_mx*params.k*omega2sin)/(-params.b*cos(theta3)+params.d_tx*params.k*sin(theta3));
                % omega_2 = sqrt((omega2sin)^2+(omega2cos)^2);
                % theta12 = atan2(omega2sin,omega2cos);

                % test 3
                theta12 = 0;
                omega_2 = 0;
                omega3_2 = 0;
                

            else

                omega3_2 = (params.d_mx*F_z_des)/(params.d_mx*params.k*sin(theta3)+params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));
                omega_2= (F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k*sin(theta12));
                omega_2_x = F_x_des/(2*params.k*cos(theta12));
                % if abs(params.k*omega_2_x -params.k*omega_2) <0.01
                %     test = 1;
                % end
                if rad2deg(theta12) < 1.904024
                    omega_2 = omega_2_x;
                end

                %omega_2 = x(21)^2;
                A = params.d_tx*params.k*sin(theta3)-params.b*cos(theta3);
                B = -2*params.d_mx*params.k;
                omega3_2 = omega_2*sin(theta12)*(B/A);
                test = A*omega3_2 -omega_2*B*sin(theta12);
                if abs(test) > 1e-10
                    flag =1;
                end
                
                % omega3_2 = (params.d_mx*F_z_des)/(params.d_mx*params.k*sin(theta3)+params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));
                % omega_2= (F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k*sin(theta12));

            end


            if omega3_2 < 0 || omega_2<0
                test = 1;
            end

            % Tx = 2*params.k*cos(x(13))*(x(21)^2);
            % theta_des = acos(F_x_des/(2*params.k*(x(21)^2)));

        end


        %check
        % term1 = -params.d_tx*params.k*omega3_2*cos(theta3);
        % term2 = -params.b*omega3_2*sin(theta3);
        % term = term1+term2;
        term3 = 2*params.d_mx*params.k*omega_2;
        term4 = params.d_tx*params.k*sin(theta3)*omega3_2;
        term5 = -params.b*cos(theta3)*omega3_2;
        term = term3+term4+term5;
        if abs(term) > 1^-10
            test =1 ;
        end
        % term6 = 2*params.k*omega_2;
        % term7 = F_z_des-params.k*sin(theta3)*omega3_2;
        % term = term6 -term7;

        u(1)=sqrt(omega_2);
        u(2)=u(1);
        u(3)=sqrt(omega3_2);
        u(4)=theta12;
        u(5)=theta12;
        u(6)=theta3;
        u(7)=-pi/2;



    case 9
        % per poter considerare azione derivativa devo considerare le velocità nel
        % frame inerziale e non body frame

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        %vx_global = V_global(1);
        %vy_global = V_global(2);
        vz_global = V_global(3);

        % controllo

        u = zeros(9,1);

        kp_z = -5;
        kd_z = -10;
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

        % COMPONENTE ORIZZONTALE

        % wind frame (NB:questo blocco di codice è anche in simulazioneVTOL2 e nelle condizioni iniziali del main)
        Va = sqrt((x(4)^2)+(x(5)^2)+(x(6)^2)); % airspeed
        %alpha = atan2(x(6),x(4)); % angle of attack
        %beta = atan2(x(5),sqrt((x(4)^2)+(x(6)^2))); % sideslip angle
        alpha = 0;
        beta = 0;
        Rwb = matriceRotazioneWingToBodyFrame(alpha,beta);

        
        vx_des = 25;
        kp_x = 5;
        ki_x = 0;
        F_x_des = (1/2)*params.rho*params.s_body_x*params.C_d_x*sign(x(4))*x(4)^2 -params.rho*params.s*(Va^2)*Rwb(1,:)*[-params.C_d;params.C_y;-params.C_l];
        F_x_des = F_x_des +kp_x*(vx_des-x(4))+ki_x*x(27);



        theta3= atan2(((-params.d_tx*params.k)/params.b),1);
        theta12 = pi/2;
        omega3_2 = (params.d_mx*F_z_des)/(params.d_mx*params.k*sin(theta3)+params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));
        omega_2= (F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k*sin(theta12));

        if omega3_2 < 0
            test = 1;
        end

        persistent flag_switching
        if isempty(flag_switching)
            flag_switching = 0;
        end

        if abs(z_des-x(3))<0.01 || x(13)~=pi/2 % inizio transizione

            kp_z = -1;
            kd_z = -5;
            z_des = -10; % asse z positivo verso il basso
            vz_des = 0; % hovering

            F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
            F_z_des = F_z_des +kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global); %PD

            % inizio ad inclinare i rotori anteriori
            eps = 10^-3; % variazioni dell'angolo
            end_angle = pi/18;%pi/36;%0.04;%0.005; % angolo terminale
            theta12 = max(x(13)-eps,end_angle);

            if  x(13)==end_angle

                test =1;
            end

            theta3 = atan2(((-params.d_tx*params.k)/params.b),1);

            Th_orizzontale_1 = params.k*cos(x(13))*x(21)^2;
            Th_orizzontale_2 = params.k*cos(x(13))*x(23)^2; 

            % supponendo che a regime theta1 = theta2 = 0 , allora il thrust
            % richiesto dai singoli rotori anteriori è dato da :
            Th_regime_orizz_i= F_x_des/2; % sarebbe il T_i_des =k*omega1^2

            errTh_x = abs(Th_orizzontale_1-Th_regime_orizz_i);

            if errTh_x < 1e-1 || flag_switching == 1

                % dopo la prima volta in cui entro (errTh_x < 1e-1)
                % continuo da lì in poi con questo controllo
                flag_switching = 1; 

                % uso la forza desiderata lungo x per calcolare vel. ang.
                % eliche anterioiri
                omega_2 = F_x_des/(2*params.k*cos(theta12));

                % calcola la vel. angol. rotore dietro per non avere pitch
                A = -2*params.d_mx*params.k*sin(theta12)*omega_2;
                B = (-params.b*cos(theta3)+params.d_tx*params.k*sin(theta3));
                omega3_2 = A/B;

                if  x(13)==end_angle

                    test =1;
                end

            else

                omega3_2 = (params.d_mx*F_z_des)/(params.d_mx*params.k*sin(theta3)+params.b*cos(theta3)-params.d_tx*params.k*sin(theta3));
                omega_2= (F_z_des-omega3_2*params.k*sin(theta3))/(2*params.k*sin(theta12));

            end

            % if abs(x(13)-end_angle) < 1e-3
            % 
            %     Th_orizzontale_1 = params.k*cos(x(13))*x(21)^2;
            %     Th_orizzontale_2 = params.k*cos(x(13))*x(23)^2;
            % 
            %     % supponendo che a regime theta1 = theta2 = 0 , allora il thrust
            %     % richiesto dai singoli rotori anteriori è dato da :
            %     Th_regime_orizz_i= F_x_des/2; % sarebbe il T_i_des =k*omega1^2
            % 
            %     omega3_2 = 0;
            %     theta12 = x(13);
            % 
            % end

        end


        %check
        % term1 = -params.d_tx*params.k*omega3_2*cos(theta3);
        % term2 = -params.b*omega3_2*sin(theta3);
        % term = term1+term2;
        term3 = 2*params.d_mx*params.k*omega_2*sin(theta12);
        term4 = params.d_tx*params.k*sin(theta3)*omega3_2;
        term5 = -params.b*cos(theta3)*omega3_2;
        term = term3+term4+term5;
        if abs(term)> 1e-10
            test=1;
        end

        term3 = 2*params.d_mx*params.k*(omega_2)*sin(x(13));
        term4 = params.d_tx*params.k*sin(theta3)*(omega3_2);
        term5 = -params.b*cos(theta3)*(omega3_2);
        term = term3+term4+term5;
        if abs(term)> 1e-10
            test=1;
        end

        term3 = 2*params.d_mx*params.k*(x(21)^2)*sin(x(13));
        term4 = params.d_tx*params.k*sin(theta3)*(x(25)^2);
        term5 = -params.b*cos(theta3)*(x(25)^2);
        term = term3+term4+term5;
        if abs(term)> 1e-10
            test=1;
        end
        % term6 = 2*params.k*omega_2;
        % term7 = F_z_des-params.k*sin(theta3)*omega3_2;
        % term = term6 -term7;

        u(1)=sqrt(omega_2);
        u(2)=u(1);
        u(3)=sqrt(omega3_2);
        u(4)=theta12;
        u(5)=theta12;
        u(6)=theta3;
        u(7)=-pi/2;




    case 10
        % 1. Estrazione Stato e Velocità Inerziali
        phi = x(7);
        theta = x(8);
        psi = x(9);
        R = matriceRotazione(phi,theta,psi);
        V_body = [x(4);x(5);x(6)];
        V_global = R*V_body ;
        vz_global = V_global(3);

        % 2. Inizializzazione Controllo
        u = zeros(7,1);
        z_des = -10;    % Target quota
        vz_des = 0;     % Target velocità (hovering)

        % 3. Parametri SMC (Sliding Mode Control)
        lambda_z = 2.5;  % Reattività convergenza errore
        K_smc = 30;      % Guadagno Robusto (Newton). Aumenta se scende troppo.
        Phi = 0.5;       % Strato limite (evita chattering)

        % 4. Calcolo Errori e Superficie
        e_z = z_des - x(3);
        de_z = vz_des - vz_global;
        s = de_z + lambda_z * e_z;    % Superficie di scorrimento

        % --- CALCOLO FORZE FISICHE (in NED) ---
        % Gravità (Positiva verso il basso)
        F_gravity = params.m * params.g * cos(x(8)) * cos(x(7));

        % Drag Aerodinamico (Se scende è opposto -> negativo. Se sale è opposto -> positivo)
        % Questa formula restituisce la forza nel frame Body/Global allineata a Z
        F_drag_aero = -params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;

        % Lift Ali (se presente)
        F_lift = -params.C_l*params.rho*params.s*x(4)^2;

        % --- LEGGE DI CONTROLLO (CALCOLO DIRETTO DEL THRUST) ---
        % Equazione: T = mg + F_aero - (Termine_Controllo)
        % Il segno meno davanti all'SMC è fondamentale:
        % Se s è negativo (devo salire), tanh è -1. -(-1) diventa +1. AUMENTA IL THRUST.

        u_controllo = params.m * lambda_z * de_z + K_smc * tanh(s / Phi);

        Thrust_req = F_gravity + F_drag_aero + F_lift - u_controllo;

        % --- MIXING ---
        theta3_ideal = atan2(((-params.d_tx*params.k)/params.b),1);
        theta3_actual = x(17);

        denom_mix = params.d_mx*params.k*sin(theta3_actual) + params.b*cos(theta3_actual) - params.d_tx*params.k*sin(theta3_actual);
        if abs(denom_mix) < 1e-6; denom_mix = 1e-6; end

        % Qui Thrust_req è già positivo e corretto
        omega3_sq = (params.d_mx * Thrust_req) / denom_mix;
        omega2_sq = (Thrust_req - omega3_sq*params.k*sin(theta3_actual)) / (2*params.k);

        % Saturazione sicurezza
        if omega2_sq < 0; omega2_sq = 0; end
        if omega3_sq < 0; omega3_sq = 0; end

        % 8. Assegnazione Output
        u(1) = sqrt(omega2_sq);      % Rotore Anteriore DX
        u(2) = u(1);                 % Rotore Anteriore SX (simmetrico)
        u(3) = sqrt(omega3_sq);      % Rotore Coda

        u(4) = pi/2;                 % Servo Ant DX
        u(5) = pi/2;                 % Servo Ant SX
        u(6) = theta3_ideal;         % Servo Coda (Target ideale)
        u(7) = -pi/2;

    case 11
        % --- CONTROLLO COMPLETO ROBUSTO (X, Y, Z) ---
        
        % 1. Estrazione Stato
        phi = x(7); theta = x(8); psi = x(9);
        p = x(10); q = x(11); r = x(12);
        
        R = matriceRotazione(phi,theta,psi); 
        V_body = [x(4);x(5);x(6)]; 
        V_global = R*V_body ;
        vx_global = V_global(1);
        vy_global = V_global(2);
        vz_global = V_global(3);

        u = zeros(7,1);

        % 2. Parametri Obiettivo
        z_des = -10;    vz_des = 0;
        y_des = 0;      vy_des = 0;
        x_des = 0;      vx_des = 0; % Vogliamo stare fermi anche su X

        % 3. Parametri Controllori
        % Z (Quota)
        lambda_z = 2.5; K_z_smc = 60; Phi_z = 0.8;
        % Y (Laterale)
        lambda_y = 0.8; K_y_smc = 15; Phi_y = 1.0;
        % X (Longitudinale)
        lambda_x = 0.8; K_x_smc = 8; Phi_x = 1.0;
        
        % PD Attitudine (Roll & Pitch)
        % Nota: Ho abbassato i guadagni come discusso per stabilità
        kp_phi = 40;   kd_phi = 8; 
        kp_theta = 40; kd_theta = 8;

        % =========================================================
        %   LOOP Z (QUOTA)
        % =========================================================
        e_z = z_des - x(3);           
        de_z = vz_des - vz_global;    
        s_z = de_z + lambda_z * e_z;    

        F_grav = params.m * params.g * cos(theta) * cos(phi); 
        F_drag_z = -params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_lift = -params.C_l*params.rho*params.s*x(4)^2;

        u_smc_z = params.m * lambda_z * de_z + K_z_smc * tanh(s_z / Phi_z);
        Thrust_req = F_grav + F_drag_z + F_lift - u_smc_z;
        if Thrust_req < 1; Thrust_req = 1; end

        % =========================================================
        %   LOOP Y (LATERALE -> ROLLIO)
        % =========================================================
        e_y = y_des - x(2);          
        de_y = vy_des - vy_global;   
        s_y = de_y + lambda_y * e_y; 
        
        F_y_req = params.m * lambda_y * de_y + K_y_smc * tanh(s_y / Phi_y);
        
        sin_phi_des = F_y_req / Thrust_req;
        sin_phi_des = max(min(sin_phi_des, 0.5), -0.5); 
        phi_des = asin(sin_phi_des);
        
        e_phi = phi_des - phi;
        de_phi = 0 - p; 
        Moment_roll_req = kp_phi * e_phi + kd_phi * de_phi;

        % =========================================================
        %   LOOP X (LONGITUDINALE -> PITCH)
        % =========================================================
        e_x = x_des - x(1);
        de_x = vx_des - vx_global;
        s_x = de_x + lambda_x * e_x;

        % Forza longitudinale richiesta
        F_x_req = params.m * lambda_x * de_x + K_x_smc * tanh(s_x / Phi_x);

        % Conversione Forza X -> Angolo Pitch
        % Attenzione ai segni: Per andare avanti (+X), serve Fx positiva.
        % Il vettore Thrust inclinato in avanti crea una Fx positiva se Theta è NEGATIVO.
        % Fx approx -Thrust * sin(theta)
        % Quindi sin(theta) = -Fx / Thrust
        sin_theta_des = -F_x_req / Thrust_req;
        sin_theta_des = max(min(sin_theta_des, 0.5), -0.5);
        theta_des = asin(sin_theta_des);

        % Controllo PD Pitch (Momento Y)
        e_theta = theta_des - theta;
        de_theta = 0 - q;
        Moment_pitch_req = kp_theta * e_theta + kd_theta * de_theta;

        % =========================================================
        %   MIXING E ALLOCAZIONE
        % =========================================================
        theta3_ideal = atan2(((-params.d_tx*params.k)/params.b),1);
        theta3_actual = x(17); 
        
        % --- 1. Mixing Longitudinale (Z + Pitch) ---
        % Qui dobbiamo bilanciare sia la Spinta Totale che il Momento di Pitch
        
        % Denominatore base (Equilibrio statico)
        denom_mix = params.d_mx*params.k*sin(theta3_actual) ...
                  - params.d_tx*params.k*sin(theta3_actual) ...
                  + params.b*cos(theta3_actual);
        if abs(denom_mix) < 1e-6; denom_mix = 1e-6; end
        
        % Modifica Cruciale: Inseriamo il Momento Pitch nel calcolo della coda
        % Se Moment_pitch_req > 0 (voglio alzare il muso), devo ridurre la coda
        % (perché la coda ha braccio negativo d_tx, quindi spinta coda crea momento giù)
        % Formula derivata:
        numeratore_coda = (params.d_mx * Thrust_req) - Moment_pitch_req;
        
        omega3_sq = numeratore_coda / denom_mix;
        
        % I motori anteriori prendono il resto della spinta verticale
        F_tail_z = omega3_sq * params.k * sin(theta3_actual);
        F_front_tot_z = Thrust_req - F_tail_z;
        omega_front_sq_base = F_front_tot_z / (2 * params.k);
        
        % --- 2. Mixing Laterale (Roll) ---
        braccio_y = params.d_my;
        delta_omega_sq = Moment_roll_req / (params.k * braccio_y * 2);
        
        omega_dx_sq = omega_front_sq_base - delta_omega_sq; 
        omega_sx_sq = omega_front_sq_base + delta_omega_sq; 
        
        % Saturazioni
        if omega_dx_sq < 0; omega_dx_sq = 0; end
        if omega_sx_sq < 0; omega_sx_sq = 0; end
        if omega3_sq < 0; omega3_sq = 0; end

        u(1) = sqrt(omega_dx_sq);    
        u(2) = sqrt(omega_sx_sq);    
        u(3) = sqrt(omega3_sq);      
        u(4) = pi/2; u(5) = pi/2; u(6) = theta3_ideal; u(7) = -pi/2;


    case 12

        u = zeros(15,1);

        % angoli 
        phi = x(7); % roll
        theta = x(8); % pitch
        psi = x(9); % yaw

        %velocità angolari
        p = x(10); % roll rate
        q = x(11); % pitch rate
        r = x(12); % yaw rate

        % angoli desiderati
        phi_des = 0;
        theta_des = 0;
        psi_des = 0;

        % errori relativi agli angoli
        e_phi = phi_des - phi;
        e_theta = theta_des - theta;
        e_psi = psi_des - psi;

        % guadagni proporzionali per la conversione :
        % da errore d'angolo a velocità angolare desiderata (rate desiderato) 
        Kr_phi = 0.1;
        Kr_theta = 0.1;
        Kr_psi = 0.1;

        % velocità angolare desiderata (rate desiderato) 
        p_t = Kr_phi*e_phi; % roll rate desiderato
        q_t = Kr_theta*e_theta; % picth rate desiderato
        r_t = Kr_psi*e_psi; % yaw rate desiderato

        % errore rate
        e_p = (p_t - p); % errore roll rate
        e_q = (q_t - q); % errore picth rate
        e_r = (r_t - r); % errore yaw rate

        % guadagni PID : roll (phi)
        Kp_phi = 0.01; % proporzionale
        Ki_phi = 0; % integrale
        Kd_phi = 0.01; % derivativo

        % guadagni PID : picth (theta)
        Kp_theta = 0.01; % proporzionale
        Ki_theta = 0; % integrale
        Kd_theta = 0.01; % derivativo

        % guadagni PID : yaw (psi)
        Kp_psi = 0.01; % proporzionale
        Ki_psi = 0; % integrale
        Kd_psi = 0.01; % derivativo

        % integrale dell'errore di rate
        I_phi = x(29); % integrale dell'errore di roll rate
        I_theta = x(30); % integrale dell'errore di pitch rate
        I_psi = x(31); % integrale dell'errore di yaw rate

        % derivata dei rate (le prendo dalle eq. differenziali del sistema)       
        global p_dot_global q_dot_global r_dot_global
        dp = p_dot_global; % derivata roll rate
        dq = q_dot_global; % derivata pitch rate
        dr = r_dot_global; % derivata yaw rate

        % nel caso in cui le variabili global non siano inizializzate
        if isempty(dp), dp = 0; end
        if isempty(dq), dq = 0; end
        if isempty(dr), dr = 0; end

        % derivata dei rate desiderati :
        % es : p_t = Kr_phi*e_phi = Kr_phi*(phi_des - phi)
        % (d/dt)(p_t) = Kr_phi*((d/dt)(phi_des) - (d/dt)(phi)))
        % poichè phi_des è costante -> (d/dt)(phi_des) = 0
        % quindi (d/dt)(p_t) = Kr_phi*(- (d/dt)(phi)))
        dp_t = -Kr_phi * p;
        dq_t = -Kr_theta * q;
        dr_t = -Kr_psi * r;


        % derivata dell'errore di rate
        de_p = dp_t - dp; % derivata dell'errore di roll rate
        de_q = dq_t - dq; % derivata dell'errore di pitch rate
        de_r = dr_t - dr; % derivata dell'errore di yaw rate


        % PID interni per controllo dell'assetto 
        u_phi = Kp_phi*e_p + Ki_phi*I_phi + Kd_phi*de_p;
        u_theta = Kp_theta*e_q + Ki_theta*I_theta + Kd_theta*de_q;
        u_psi = Kp_psi*e_r + Ki_psi*I_psi + Kd_psi*de_r;

        u_phi = 0;
        u_theta = 0;
        u_psi = 0;

        % Controllo verticale
        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        vz_global = V_global(3);

        % posizione e velocità desiderate
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        %guadagni PD
        % kp_z = -5;
        % kd_z = -5;
        % ki_z = -0.01;

        kp_z = -3;
        kd_z = -15;
        ki_z = 0;

        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        
        % PD
        u_PID = kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global)+ki_z*x(35); 
        u_t = sqrt((F_z_des)/(3*params.k))+u_PID;

        u(1) = -u_phi + u_theta +u_t;
        u(2) = u_phi + u_theta +u_t;
        u(3) = -u_theta +u_t;
        u(4) = pi/2 - u_psi;
        u(5) = pi/2 + u_psi;
        u(6) = pi/2;
        u(7) = 0;

        % per il calcolo dell'integrale dell'errore di rate
        u(10) = e_p; % errore roll rate
        u(11) = e_q; % errore pitch rate
        u(12) = e_r; % errore yaw rate

        u(16) = z_des-x(3);

        
    case 13

        u = zeros(15,1);

        % angoli 
        phi = x(7); % roll
        theta = x(8); % pitch
        psi = x(9); % yaw

        %velocità angolari
        p = x(10); % roll rate
        q = x(11); % pitch rate
        r = x(12); % yaw rate

        % angoli desiderati
        phi_des = 0;
        theta_des = 0;
        psi_des = 0;

        % errori relativi agli angoli
        e_phi = phi_des - phi;
        e_theta = theta_des - theta;
        e_psi = psi_des - psi;

        % velocità angolare desiderata (rate desiderato) 
        p_des = 0; % roll rate desiderato
        q_des = 0; % picth rate desiderato
        r_des = 0; % yaw rate desiderato

        Omega_body = [p;q;r];
        J = matriceJ(phi,theta,psi); % matrice di trasformazione  : OmegaVtol_body (p,q,r) -> Omega_global (phi_dot,theta_dot,psi_dot)
        Omega = J*Omega_body;

        % errore rate
        e_p = (p_des - Omega(1)); % errore roll rate
        e_q = (q_des - Omega(2)); % errore picth rate
        e_r = (r_des - Omega(3)); % errore yaw rate

        % guadagni PID : roll (phi)
        Kp_phi = 0.1; % proporzionale
        Ki_phi = 0; % integrale
        Kd_phi = 0; % derivativo

        % guadagni PID : picth (theta)
        Kp_theta = -0.1; % proporzionale
        Ki_theta = 0; % integrale
        Kd_theta = 0; % derivativo

        % guadagni PID : yaw (psi)
        Kp_psi = 0.1; % proporzionale
        Ki_psi = 0; % integrale
        Kd_psi = 0; % derivativo

        % integrale dell'errore di rate
        I_phi = x(29); % integrale dell'errore di roll
        I_theta = x(30); % integrale dell'errore di pitch
        I_psi = x(31); % integrale dell'errore di yaw

        % PID interni per controllo dell'assetto 
        u_phi = Kp_phi*e_phi + Ki_phi*I_phi + Kd_phi*e_p;
        u_theta = Kp_theta*e_theta + Ki_theta*I_theta + Kd_theta*e_q;
        u_psi = Kp_psi*e_psi + Ki_psi*I_psi + Kd_psi*e_r;


        % Controllo verticale
        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        vz_global = V_global(3);

        % posizione e velocità desiderate
        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        %guadagni PD
        kp_z = -3;
        kd_z = -15;
        ki_z = 0;

        % Thrust (totale) necessario per la compensazione
        F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        
        % Thrust (dei singoli rotori )necessario per la compensazione
        T_i = F_z_des/3;

        % velocità angolare (dei singoli rotori) per garantire
        % compensazione (è la mia omega di base)
        omega1 = sqrt((T_i)/(params.k*sin(x(13))));
        omega2 = sqrt((T_i)/(params.k*sin(x(15))));
        omega3 = sqrt((T_i)/(params.k*sin(x(17))));

        % PD
        u_PID = kp_z*(z_des-x(3))+kd_z*(vz_des-vz_global)+ki_z*x(35); 
        
        u_t_1 = omega1 + u_PID;
        u_t_2 = omega2 + u_PID;
        u_t_3 = omega3+ u_PID;

        u(1) = -u_phi + u_theta +u_t_1;
        u(2) = u_phi + u_theta +u_t_2;
        u(3) = -u_theta +u_t_3;
        u(4) = pi/2 - u_psi;
        u(5) = pi/2 + u_psi;
        u(6) = pi/2;
        u(7) = 0;
        

        % per il calcolo dell'integrale dell'errore 
        u(10) = e_phi; % errore roll 
        u(11) = e_theta; % errore pitch 
        u(12) = e_psi; % errore yaw 
        u(16) = z_des-x(3);

        
    case 14

        u = zeros(16,1);

        % angoli 
        phi = x(7); % roll
        theta = x(8); % pitch
        psi = x(9); % yaw

        %velocità angolari
        p = x(10); % roll rate
        q = x(11); % pitch rate
        r = x(12); % yaw rate

        % --- CONTROLLO VERTICALE  + ORIENTAMENTO  ---

        V_body = [x(4);x(5);x(6)];
        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_global = R*V_body ;
        vx_global = V_global(1); vy_global = V_global(2); vz_global = V_global(3);
 

        % PD Attitudine (Roll & Pitch)
        kp_phi = 15;   kd_phi = 3; ki_phi = 0.05;
        kp_theta = 15; kd_theta = 3; ki_theta = 0.05;

        % PD Yaw (Imbardata)
        kp_psi = 15;   kd_psi = 3; ki_psi = 0.005;

        % =========================================================
        %   QUOTA - HOVERING (PD/PID)
        % =========================================================

        z_des = -10; % asse z positivo verso il basso
        vz_des = 0; % hovering

        e_z = z_des - x(3);
        de_z = vz_des - vz_global;

        % per integrale dell'errore di posizione lungo z
        u(15) = e_z;

        kp_z = -5;
        kd_z = -10;
        ki_z = -0.005;
        
        %F_z_des = -params.C_l*params.rho*params.s*x(4)^2 +params.m*params.g*cos(x(8))*cos(x(7)) - params.rho*params.s_body_z*params.C_d_z*sign(x(6))*x(6)^2;
        F_z_des = params.m*params.g*cos(x(8))*cos(x(7)); % compenso solo la gravità
        %Thrust_req = F_z_des +kp_z*(e_z)+kd_z*(de_z); %PD
        Thrust_req = F_z_des +kp_z*(e_z)+kd_z*(de_z) +ki_z*x(34); %PID

        % =========================================================
        %   ROLL (PD/PID)
        % =========================================================

        % angolo richiesto per compensare inclinazione rotore posteriore
        % num_roll = -cos(x(17))*sin(x(19))*params.k*(x(23)^2);
        % den_roll = cos(x(8))*params.m*params.g;
        % roll = asin(num_roll/den_roll);
        

        y_des = 0;
        e_y = y_des - x(2);
        
        vy_des = 0;
        e_vy = vy_des - vy_global;

        % per integrale dell'errore di posizione lungo y
        u(14) = e_y;

        kp_y = 0.5;    
        kp_vy = 0.5;  
        ki_y = 0.05;

        %phi_cmd = kp_y * e_y +kp_vy * e_vy; % PD
        phi_cmd = kp_y * e_y +kp_vy * e_vy +ki_y*x(33);% PID

        %phi_des = roll + phi_cmd; 
        phi_des = phi_cmd;

        % saturazione roll desiderato
        phi_max = deg2rad(20);
        phi_des = max(min(phi_des, phi_max), -phi_max);

        e_phi = phi_des - phi;
        de_phi = 0 - p;

        %Moment_roll_req = kp_phi * e_phi + kd_phi * de_phi; % (PD)
        Moment_roll_req = kp_phi * e_phi + kd_phi * de_phi + ki_phi*x(29); % (PID)

        % per integrale dell'errore di roll
        u(10) = e_phi;

        % =========================================================
        %   PITCH (PD/PID)
        % =========================================================

        x_des = 0;
        e_x = x_des - x(1);

        vx_des = 0;
        e_vx = vx_des - vx_global;

        % per integrale dell'errore di posizione lungo x
        u(13) = e_x;

        kp_x = 0.5;
        kp_vx = 0.5;
        ki_x = 0.05;

        %ax_des = kp_x * e_x + kp_vx * e_vx; %PD
        ax_des = kp_x * e_x + kp_vx * e_vx +ki_x*x(32); %PID

        theta_cmd = -ax_des;

        % saturazione pitch desiderato
        theta_max = deg2rad(20);
        theta_cmd = max(min(theta_cmd, theta_max), -theta_max);

        theta_des = theta_cmd;
        %theta_des = 0;

        e_theta = theta_des - theta;
        de_theta = 0 - q;

        %Moment_pitch_req = kp_theta * e_theta + kd_theta * de_theta; %(PD)
        Moment_pitch_req = kp_theta * e_theta + kd_theta * de_theta + ki_theta*x(30); % (PID)

        % per integrale dell'errore di pitch
        u(11) = e_theta;

        % =========================================================
        %   YAW (PD/PID)
        % =========================================================
        psi_des = 0;

        e_psi = psi_des - psi;
        de_psi = 0 - r;

        %Moment_yaw_req = kp_psi * e_psi + kd_psi * de_psi; % (PD)
        Moment_yaw_req = kp_psi * e_psi + kd_psi * de_psi + ki_psi*x(31); %(PID)

        % per integrale dell'errore di yaw
        u(12) = e_psi;

        % =========================================================
        %   MOTOR/TILT MIXING ALGORITHM
        % =========================================================
        theta3_ideal = atan2(((-params.d_tx*params.k)/params.b),1);
        theta3_actual = x(17);

        % --- 1. Mixing (Z + Pitch) ---
        denom_mix = params.d_mx*params.k*sin(theta3_actual) ...
            - params.d_tx*params.k*sin(theta3_actual) ...
            + params.b*cos(theta3_actual);


        numeratore_coda = (params.d_mx * Thrust_req) - Moment_pitch_req;
        omega3_sq = numeratore_coda / denom_mix;

        F_tail_z = omega3_sq * params.k * sin(theta3_actual);

        % Spinta totale richiesta ai motori anteriori (componente Z)
        F_front_tot_z = Thrust_req - F_tail_z;

        omega_front_sq_base = F_front_tot_z / (2 * params.k*sin(x(13)));

        % --- 2. Mixing Roll ---
        braccio_y = params.d_my;
        delta_omega_sq = Moment_roll_req / (params.k * braccio_y * 2*sin(x(13)));

        omega_dx_sq = omega_front_sq_base - delta_omega_sq;
        omega_sx_sq = omega_front_sq_base + delta_omega_sq;

        % --- 3. Mixing Yaw ---

        delta_tilt_yaw = pi/2 - atan2(F_front_tot_z * params.d_my,Moment_yaw_req);

        % Saturazione del tilt per sicurezza (es. max 20 gradi = 0.35 rad)
        max_tilt = 0.35;

        delta_tilt_yaw = max(min(delta_tilt_yaw, max_tilt), -max_tilt);

        % Assegnazione Tilt (Partendo da pi/2 verticale)
        tilt_1 = pi/2 + delta_tilt_yaw; % Motore DX (1)
        tilt_2 = pi/2 - delta_tilt_yaw; % Motore SX (2)

        % Saturazioni Motori
        if omega_dx_sq < 0; omega_dx_sq = 0; end
        if omega_sx_sq < 0; omega_sx_sq = 0; end
        if omega3_sq < 0; omega3_sq = 0; end

        u(1) = sqrt(omega_dx_sq);
        u(2) = sqrt(omega_sx_sq);
        u(3) = sqrt(omega3_sq);
        u(4) = tilt_1;  % Tilt Destro
        u(5) = tilt_2;  % Tilt Sinistro
        u(6) = theta3_ideal;
        u(7) = -pi/2;


    case 15

        % angoli di roll ,pitch, yaw
        phi = x(7);
        theta = x(8);
        psi = x(9);

        R = matriceRotazione(phi,theta,psi); % matrice di rotazione
        V_body = [x(4);x(5);x(6)]; % velocità nel body frame
        V_global = R*V_body ;
        vx_global = V_global(1);

        % wind frame 
        Va = sqrt((x(4)^2)+(x(5)^2)+(x(6)^2)); % airspeed
        %alpha = atan2(x(6),x(4)); % angle of attack
        %beta = atan2(x(5),sqrt((x(4)^2)+(x(6)^2))); % sideslip angle
        alpha = 0;
        beta = 0;
        Rwb = matriceRotazioneWingToBodyFrame(alpha,beta);

        % controllo

        u = zeros(16,1);

        vx_des = 25;

        kp_x = 5;

        F_x_des = (1/2)*params.rho*params.s_body_x*params.C_d_x*sign(x(4))*x(4)^2 -params.rho*params.s*(Va^2)*Rwb(1,:)*[-params.C_d;params.C_y;-params.C_l];
        F_x_des = F_x_des +kp_x*(vx_des-vx_global);

        % Assegna ai due rotori anteriori
        T_i = F_x_des/2;

        u(1) = sqrt(T_i/params.k);   % omega1
        u(2) = u(1);                 % omega2
        u(3) = 0;                    % omega3 -> rotore di coda spento
        u(4) = 0;             % tilt rotore 1
        u(5) = 0;             % tilt rotore 2
        u(6) = 0;
        u(7) = 0;



    otherwise
        error('Controllo non valido');
end

end