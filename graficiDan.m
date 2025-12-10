%% Frequenza cardiaca sintetica realistica - Calciatore
clear; clc; close all;

rng(4); % per riproducibilità

%% Parametri base
fs = 1;                % 1 Hz (1 campione al secondo)
dt = 1/fs;

T_half = 45*60;        % 45 minuti in secondi
T_ht   = 15*60;        % intervallo
T_match = 2*T_half + T_ht;

t = (0:dt:T_match-dt)'; % asse tempi

%% Stati di intensità
% 1=cammino/pausa  2=jog  3=corsa intensa  4=sprint/azioni massimali
state_names = ["Walk/Low","Jog","Run","Sprint"];

% HR tipiche (target) per ciascuno stato (valori plausibili)
mu = [125, 145, 165, 182];  % bpm medi target
sd = [  6,   6,   5,   4];  % variabilità target

% Matrice di transizione (più probabile restare nello stesso stato
% o passare a stati adiacenti)
P = [0.84 0.14 0.02 0.00;
     0.12 0.75 0.12 0.01;
     0.03 0.15 0.75 0.07;
     0.00 0.05 0.20 0.75];

%% Funzione per simulare una metà
simulate_half = @(N, fatigue_shift) ...
    simulateIntensityStates(N, P, mu + fatigue_shift, sd);

%% Primo tempo
N1 = T_half * fs;
[st1, HRtarg1] = simulate_half(N1, 0);

%% Intervallo (HR scende verso valori più bassi)
Nht = T_ht * fs;
stHT = ones(Nht,1); % stato "low"
HRtargHT = 108 + 4*randn(Nht,1);

%% Secondo tempo con leggera fatica (shift +3 bpm)
N2 = T_half * fs;
[st2, HRtarg2] = simulate_half(N2, 3);

%% Concatena target e stati
st = [st1; stHT; st2];
HR_targ = [HRtarg1; HRtargHT; HRtarg2];

%% Modello dinamico della risposta cardiaca
HR = zeros(size(HR_targ));
HR(1) = 95; % valore di partenza plausibile pre-inizio

tau_up = 18;   % s: salita abbastanza rapida
tau_dn = 32;   % s: discesa più lenta

noise_sd = 0.35; % rumore fisiologico piccolo

for k = 1:length(HR)-1
    e = HR_targ(k) - HR(k);

    if e >= 0
        tau = tau_up;
    else
        tau = tau_dn;
    end

    HR(k+1) = HR(k) + (dt/tau)*e + noise_sd*randn;
end

%% Limiti realistici
HR = min(max(HR, 90), 195);

%% Grafico (stile rosso su nero)
figure('Color','k');
ax = axes('Color','k');
hold(ax,'on');

% Area rossa sotto la curva (look simile al tuo esempio)
A = area(ax, t/60, HR);
A.FaceColor = [1 0 0];
A.EdgeColor = 'none';
A.FaceAlpha = 0.25;

% Linea principale
plot(ax, t/60, HR, 'LineWidth', 2, 'Color', [1 0.35 0.35]);

% Griglia e colori assi
grid(ax,'on');
ax.XColor = [0.85 0.85 0.85];
ax.YColor = [0.85 0.85 0.85];
ax.GridColor = [0.5 0.5 0.5];
ax.GridAlpha = 0.2;
ax.Box = 'off';

xlabel(ax, 'Tempo (min)', 'Color', [0.85 0.85 0.85]);
ylabel(ax, 'Frequenza cardiaca (bpm)', 'Color', [0.85 0.85 0.85]);
title(ax, 'Frequenza cardiaca - Calciatore 1', 'Color', [0.95 0.95 0.95]);

% Evidenzia intervallo
xline(ax, 45, '--', 'Fine 1° tempo', 'Color', [0.8 0.8 0.8]);
xline(ax, 60, '--', 'Inizio 2° tempo', 'Color', [0.8 0.8 0.8]);

ylim(ax, [90 195]);

%% (Opzionale) Plot degli stati di intensità sotto
% figure('Color','k');
% ax2 = axes('Color','k');
% stairs(ax2, t/60, st, 'LineWidth', 1.2, 'Color', [0.85 0.85 0.85]);
% yticks(ax2, 1:4);
% yticklabels(ax2, state_names);
% grid(ax2, 'on');
% ax2.XColor = [0.85 0.85 0.85];
% ax2.YColor = [0.85 0.85 0.85];
% ax2.GridColor = [0.5 0.5 0.5];
% ax2.GridAlpha = 0.2;
% xlabel(ax2, 'Tempo (min)', 'Color', [0.85 0.85 0.85]);
% ylabel(ax2, 'Intensità (stato)', 'Color', [0.85 0.85 0.85]);
% title(ax2, 'Sequenza di intensità simulata', 'Color', [0.95 0.95 0.95]);

%% ================= FUNZIONE LOCALE =================
function [st, HRtarg] = simulateIntensityStates(N, P, mu, sd)
    st = zeros(N,1);
    HRtarg = zeros(N,1);

    % stato iniziale più probabile: jog/low
    st(1) = randsample(1:4, 1, true, [0.35 0.40 0.20 0.05]);

    for k = 2:N
        st(k) = randsample(1:4, 1, true, P(st(k-1),:));
    end

    % genera HR target per stato
    for k = 1:N
        HRtarg(k) = mu(st(k)) + sd(st(k))*randn;
    end

    % piccole ondulazioni lente (ritmo partita)
    drift = 1.5*sin(2*pi*(1:N)'/(7*60)) + 0.8*sin(2*pi*(1:N)'/(3.5*60));
    HRtarg = HRtarg + drift;
end
