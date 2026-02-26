% === Inizio del tuo script MATLAB ===
clc;
clear;

% --- CARICAMENTO DEI TUOI DATI REALI DA SIMULINK ---
% QUESTE RIGHE SONO ESEMPI. DEVI SOSTITUIRLE CON IL METODO CHE USI PER CARICARE
% IL TEMPO (t_sim), L'INPUT Y AL CORDIC (Y_input_simulink)
% E L'OUTPUT DELL'ANGOLO DAL CORDIC (theta_cordic_output) DALLA TUA SIMULAZIONE.
%
% Esempio (adatta i nomi delle variabili al tuo modello Simulink):
% load('i_miei_dati_simulink_arcoseno.mat');
% t_sim = out.logs.Time;              % Esempio: vettore del tempo
% Y_input_simulink = out.logs.Y_input.Data; % Esempio: il tuo input Y (quello "sigmoide")
% theta_cordic_output = out.logs.Cordic_Angle_Out.Data; % Esempio: l'angolo di output del CORDIC


% --- DATI SINTETICI (SOLO PER DIMOSTRAZIONE - RIMUOVI O COMMENTA NELLA TUA VERSIONE FINALE) ---
% Se non hai i dati subito, puoi usare questi per vedere come funziona il grafico.
num_points = 1000;
t_sim = linspace(0, 7, num_points); % Vettore del tempo da 0 a 7 secondi
% L'angolo reale che il CORDIC deve trovare. Facciamolo variare nel dominio corretto per arcsin.
theta_reale_per_input_rad = linspace(-pi/2, pi/2, num_points);
Y_input_simulink = sin(theta_reale_per_input_rad); % L'input 'y' al CORDIC (valori da -1 a 1)
% Simula l'output del CORDIC, aggiungendo un piccolo errore per imitare il Fixed-Point
theta_cordic_output = asin(Y_input_simulink) + (rand(size(Y_input_simulink)) - 0.5) * 5e-3; % asin ideale + rumore fixed-point
% Clampa l'output del CORDIC nel range corretto per asin
theta_cordic_output = max(-pi/2, min(pi/2, theta_cordic_output));
% --- FINE DATI SINTETICI ---


% --- CALCOLO DELL'ANGOLO REALE ---
% L'angolo reale (ground truth) è semplicemente asin(Y_input_simulink)
theta_reale_rad = asin(Y_input_simulink);

% Converti gli angoli da radianti a gradi per una migliore leggibilità del grafico
theta_reale_gradi = rad2deg(theta_reale_rad);
theta_cordic_output_gradi = rad2deg(theta_cordic_output);


% --- GENERAZIONE DEL GRAFICO ---
figure('Color', 'w', 'Position', [100 100 800 500]); % Imposta dimensione e posizione della figura

% Grafico: Confronto Angolo Reale vs. Angolo Calcolato da CORDIC
plot(t_sim, theta_reale_gradi, 'b-', 'LineWidth', 2, 'DisplayName', 'Angolo Reale $\theta_{reale}$'); hold on;
plot(t_sim, theta_cordic_output_gradi, 'r--', 'LineWidth', 2, 'DisplayName', 'Angolo Calcolato $\theta_{CORDIC}$');
title('Confronto: Angolo Reale vs. Angolo Calcolato da CORDIC (Modalità Arcoseno)');
xlabel('Tempo (s)');
ylabel('Angolo (Gradi)');
legend('Location', 'best', 'Interpreter', 'latex', 'Box', 'off'); % Legenda posizionata al meglio
grid on;
hold off;

% Salva il grafico (opzionale)
print('confronto_arcoseno_cordic_diretto.png', '-dpng', '-r300');