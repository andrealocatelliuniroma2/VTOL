%% Frequenza cardiaca sintetica realistica - Calciatore (con eventi + goal + recupero)
clear; clc; close all;

rng(10); % cambia questo numero per un match diverso

%% Parametri base
fs = 1;                % 1 Hz (1 campione al secondo)
dt = 1/fs;

% -----------------------------
% RECUPERO (in minuti)
% -----------------------------
rec1_min = 2;   % recupero 1° tempo (es. 0,1,2,3,4...)
rec2_min = 5;   % recupero 2° tempo

T_half1  = (45 + rec1_min)*60;   % 1° tempo in secondi
T_ht     = 15*60;                % intervallo
T_half2  = (45 + rec2_min)*60;   % 2° tempo in secondi

T_match = T_half1 + T_ht + T_half2;

t = (0:dt:T_match-dt)'; % asse tempi "reali", include intervallo e recuperi

%% Stati di intensità
% 1=cammino/pausa  2=jog  3=corsa intensa  4=sprint/azioni massimali
state_names = ["Walk/Low","Jog","Run","Sprint"];

% HR tipiche (target) per ciascuno stato (valori plausibili)
mu = [125, 145, 165, 182];  % bpm medi target
sd = [  6,   6,   5,   4];  % variabilità target

% Portiere: baseline più basso, picchi più rari
% mu = [112, 124, 145, 165];
% sd = [  5,   5,   5,   4];

%% -----------------------------
% VARIABILITÀ TRA PARTITE
% -----------------------------
match_shift = 5*randn;          % sposta tutta la HR di qualche bpm
mu = mu + match_shift;

sd_scale = 0.9 + 0.88*rand;      % più o meno "nervosa" la partita
sd = sd .* sd_scale;

%% Matrice di transizione (base)
P_base = [0.84 0.14 0.02 0.00;
          0.12 0.75 0.12 0.01;
          0.03 0.15 0.75 0.07;
          0.00 0.05 0.20 0.75];

%% -----------------------------
% LEGGERA RANDOMIZZAZIONE DI P
% -----------------------------
P = P_base + 0.01*randn(size(P_base));
P(P < 0) = 0;
row_sums = sum(P,2);
row_sums(row_sums == 0) = 1;
P = P ./ row_sums;

%% Funzione per simulare una metà
simulate_half = @(N, fatigue_shift) ...
    simulateIntensityStates(N, P, mu + fatigue_shift, sd);

%% Primo tempo (con recupero)
N1 = T_half1 * fs;
[st1, HRtarg1] = simulate_half(N1, 0);

%% Intervallo (HR scende verso valori più bassi)
Nht = T_ht * fs;
stHT = ones(Nht,1); % stato "low"
HRtargHT = 108 + 4*randn(Nht,1);

%% Secondo tempo (con recupero)
N2 = T_half2 * fs;
[st2, HRtarg2] = simulate_half(N2, 3);

%% Concatena target e stati
st = [st1; stHT; st2];
HR_targ = [HRtarg1; HRtargHT; HRtarg2];

% -----------------------------
% MACRO-FASI DI PARTITA (5-12 min)
% -----------------------------
N = length(HR_targ);
nPhases = randi([6 10]);

edges = round(linspace(1, N, nPhases+1));
jitter = round(0.02*N*randn(size(edges)));
edges = max(1, min(N, edges + jitter));
edges(1) = 1; edges(end) = N;
edges = sort(unique(edges));
if numel(edges) < 3
    edges = [1; round(N/2); N];
end

phase_shift = -4 + 10*rand(numel(edges)-1,1);

for i = 1:numel(edges)-1
    idx = edges(i):edges(i+1);
    HR_targ(idx) = HR_targ(idx) + phase_shift(i);
end

%% -----------------------------
% EVENTI INTENSI CASUALI
% -----------------------------
nEvents = randi([8 16]);
for i = 1:nEvents
    dur = randi([8 25]);
    idx = randi([1, length(HR_targ)-dur-1]);
    bump = 6 + 6*rand;
    HR_targ(idx:idx+dur) = HR_targ(idx:idx+dur) + bump;
end

%% -----------------------------
% EVENTO GOAL
% Inserisci qui i minuti "da tabellino"
% NOTA: con recupero, puoi inserire anche 46..(90+rec2_min)
% Esempio: goal al 4', 25', 43', 90' ecc.
% -----------------------------
goal_match_minutes = [77];  % <-- MODIFICA QUI (puoi anche mettere [])

% Parametri picco goal
% (Se vuoi valori realistici: prova goal_amp_base ~ 8-14)
goal_amp_base = 80;
goal_amp_var  = 4;
goal_dur_base = 120;
goal_dur_var  = 40;
tau_goal_base = 25;
tau_goal_var  = 10;

first_half_end_min = 45 + rec1_min;  % fine 1° tempo sul clock partita

for g = 1:length(goal_match_minutes)
    gm = goal_match_minutes(g);

    % Mappa minuto partita -> timeline che include intervallo
    % Se il goal è oltre la fine del 1° tempo (con recupero),
    % sulla timeline reale va aggiunto l'intervallo di 15'
    if gm <= first_half_end_min
        goal_timeline_min = gm;
    else
        goal_timeline_min = gm + 15;
    end

    idx0 = round(goal_timeline_min*60*fs) + 1;
    if idx0 < 1 || idx0 > length(HR_targ)
        continue;
    end

    goal_amp = goal_amp_base + goal_amp_var*rand;
    goal_dur = goal_dur_base + randi(goal_dur_var);
    tau_goal = tau_goal_base + tau_goal_var*rand;

    idx1 = min(idx0 + goal_dur, length(HR_targ));

    n = (0:(idx1-idx0))';
    pulse = goal_amp * exp(-n/tau_goal);

    if gm > (75) % puoi anche cambiare questa soglia
        pulse = pulse + 2;
    end

    HR_targ(idx0:idx1) = HR_targ(idx0:idx1) + pulse;
end

%% Modello dinamico della risposta cardiaca
HR = zeros(size(HR_targ));
HR(1) = 95;

tau_up = 18;
tau_dn = 32;

noise_sd = 0.55;

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

A = area(ax, t/60, HR);
A.FaceColor = [1 0 0];
A.EdgeColor = 'none';
A.FaceAlpha = 0.25;

plot(ax, t/60, HR, 'LineWidth', 2, 'Color', [1 0.35 0.35]);

grid(ax,'on');
ax.XColor = [0.85 0.85 0.85];
ax.YColor = [0.85 0.85 0.85];
ax.GridColor = [0.5 0.5 0.5];
ax.GridAlpha = 0.2;
ax.Box = 'off';

xlabel(ax, 'Tempo (min)', 'Color', [0.85 0.85 0.85]);
ylabel(ax, 'Frequenza cardiaca (bpm)', 'Color', [0.85 0.85 0.85]);
title(ax, 'Frequenza cardiaca - Calciatore 5', 'Color', [0.95 0.95 0.95]);

% Evidenzia fine 1° tempo e inizio 2° tempo (con recupero)
end1_min_timeline = first_half_end_min;       % sulla timeline reale
start2_min_timeline = first_half_end_min + 15;

xline(ax, end1_min_timeline, '--', 'Fine 1° tempo', 'Color', [0.8 0.8 0.8]);
xline(ax, start2_min_timeline, '--', 'Inizio 2° tempo', 'Color', [0.8 0.8 0.8]);

ylim(ax, [90 195]);

%% ================= FUNZIONE LOCALE =================
function [st, HRtarg] = simulateIntensityStates(N, P, mu, sd)
    st = zeros(N,1);
    HRtarg = zeros(N,1);

    st(1) = randsample(1:4, 1, true, [0.35 0.40 0.20 0.05]);

    for k = 2:N
        st(k) = randsample(1:4, 1, true, P(st(k-1),:));
    end

    for k = 1:N
        HRtarg(k) = mu(st(k)) + sd(st(k))*randn;
    end

    drift = 1.5*sin(2*pi*(1:N)'/(7*60)) + 0.8*sin(2*pi*(1:N)'/(3.5*60));
    HRtarg = HRtarg + drift;
end
