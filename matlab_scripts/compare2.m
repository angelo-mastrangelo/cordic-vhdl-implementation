% CORDIC - Confronto Errore CORDIC vs Precisione di MATLAB (Tentativo di Correzione Finale)
clear; clc;
% --- Parametri CORDIC ---
n_iterations_cordic = 32; 
% Calcolo della tabella atan_table e del fattore K
atan_table_cordic = atan(2.^(-(0:n_iterations_cordic-1)));
k_factor_cordic = prod(1 ./ sqrt(1 + 2.^(-2*(0:n_iterations_cordic-1))));
% --- Dati di test ---
% Angoli per sin (da 0° a 360°, passo 0.1°)
theta_values_deg = 0:0.1:360;
theta_values_rad = deg2rad(theta_values_deg);
% Input per asin (da -1 a 1 con passo 0.01)
x_values_asin = -1:0.01:1;
% Coppie y/x per atan2 (per coprire l'intero range di angoli da 0 a 360 gradi)
num_points_atan = 3601; % 0.1 gradi per 360 gradi, +1 per 0
test_angles_atan_deg_plot = linspace(0, 360, num_points_atan); % Per l'asse X del plot
test_angles_atan_rad_calc = deg2rad(test_angles_atan_deg_plot); % Angoli in radianti per il calcolo
y_values_atan = sin(test_angles_atan_rad_calc);
x_values_atan = cos(test_angles_atan_rad_calc);
% --- Precisione macchina di MATLAB ---
matlab_precision_limit = eps; 
% --- Preallocazione per gli errori ---
errors_cordic_sin = zeros(1, length(theta_values_rad));
errors_cordic_asin = zeros(1, length(x_values_asin));
errors_cordic_atan = zeros(1, length(y_values_atan));
%% === Calcolo degli Errori CORDIC ===
% --- SIN ---
for t_idx = 1:length(theta_values_rad)
    theta_rad = theta_values_rad(t_idx);
    
    cordic_result_sin = cordic_sin_function(theta_rad, n_iterations_cordic, k_factor_cordic, atan_table_cordic);
    
    matlab_result = sin(theta_rad);
    errors_cordic_sin(t_idx) = max(matlab_precision_limit, abs(cordic_result_sin - matlab_result));
end
% --- ASIN ---
for x_idx = 1:length(x_values_asin)
    x = x_values_asin(x_idx);
    
    if abs(x) > 1 
        errors_cordic_asin(x_idx) = NaN; 
        continue;
    end
    
    cordic_result_asin = cordic_asin_function(x, n_iterations_cordic, atan_table_cordic);
    matlab_result = asin(x);
    errors_cordic_asin(x_idx) = max(matlab_precision_limit, abs(cordic_result_asin - matlab_result));
end
% --- ATAN ---
for yx_idx = 1:length(y_values_atan)
    y = y_values_atan(yx_idx);
    x = x_values_atan(yx_idx);
    
    cordic_result_atan = cordic_atan_function_map_0_2pi(y, x, n_iterations_cordic, atan_table_cordic);
    
    % Mappa anche il risultato MATLAB atan2 a [0, 2*pi] per confronto
    matlab_result = atan2(y, x);
    if matlab_result < 0
        matlab_result = matlab_result + 2*pi;
    end
    errors_cordic_atan(yx_idx) = max(matlab_precision_limit, abs(cordic_result_atan - matlab_result));
end
%% === Generazione dei Grafici ===
output_folder = 'grafici_confronto_errori';
if ~exist(output_folder, 'dir'); mkdir(output_folder); end
% --- GRAFICO SIN ---
figure;
% Disegna prima la linea CORDIC con colore ROSSO ('r-')
semilogy(theta_values_deg, errors_cordic_sin, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Errore CORDIC'); hold on;
% Poi disegna la linea Precisione MATLAB
semilogy(theta_values_deg, ones(size(theta_values_deg)) * matlab_precision_limit, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Precisione MATLAB (eps)'); 
xlabel('Angolo (Gradi)', 'FontSize', 12);
ylabel('Errore Assoluto (log_{10})', 'FontSize', 12);
title(['Confronto Errore - Funzione SIN'], 'FontSize', 14);
legend('Location', 'NorthEast', 'FontSize', 12); 
grid on;
ylim([1e-18 1e-0]); 
xlim([0 360]); % IMPOSTA I LIMITI DELL'ASSE X DA 0 A 360 GRADI
set(gca, 'FontSize', 12);
saveas(gcf, fullfile(output_folder, 'sin_error_comparison.png'));
% --- GRAFICO ASIN ---
figure;
valid_indices_asin = ~isnan(errors_cordic_asin);
plot_x_values_asin = x_values_asin(valid_indices_asin);
plot_errors_cordic_asin = errors_cordic_asin(valid_indices_asin);
% Disegna prima la linea CORDIC con colore VERDE ('g-')
semilogy(plot_x_values_asin, plot_errors_cordic_asin, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Errore CORDIC'); hold on;
% Poi disegna la linea Precisione MATLAB
semilogy(plot_x_values_asin, ones(size(plot_x_values_asin)) * matlab_precision_limit, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Precisione MATLAB (eps)'); 
xlabel('Valore X (Input asin)', 'FontSize', 12);
ylabel('Errore Assoluto (log_{10})', 'FontSize', 12);
title(['Confronto Errore - Funzione ASIN'], 'FontSize', 14);
legend('Location', 'NorthEast', 'FontSize', 12); 
grid on;
ylim([1e-18 1e-0]); 
% xlim([-1 1]); % L'asse X di ASIN è già correttamente tra -1 e 1
set(gca, 'FontSize', 12);
saveas(gcf, fullfile(output_folder, 'asin_error_comparison.png'));
% --- GRAFICO ATAN ---
figure;
% Disegna prima la linea CORDIC con colore BLU ('b-')
semilogy(test_angles_atan_deg_plot, errors_cordic_atan, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Errore CORDIC'); hold on;
% Poi disegna la linea Precisione MATLAB
semilogy(test_angles_atan_deg_plot, ones(size(test_angles_atan_deg_plot)) * matlab_precision_limit, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Precisione MATLAB (eps)'); 
xlabel('Angolo (Gradi)', 'FontSize', 12);
ylabel('Errore Assoluto (log_{10})', 'FontSize', 12);
title(['Confronto Errore - Funzione ATAN'], 'FontSize', 14);
legend('Location', 'NorthEast', 'FontSize', 12); 
grid on;
ylim([1e-18 1e-0]); 
xlim([0 360]); % IMPOSTA I LIMITI DELL'ASSE X DA 0 A 360 GRADI
set(gca, 'FontSize', 12);
saveas(gcf, fullfile(output_folder, 'atan_error_comparison.png'));
%% === FUNZIONI CORDIC (revisionate per robustezza e precisione) ===
% Funzione ausiliaria CORDIC per la rotazione (base per sin/cos)
function [x_out, y_out, z_out] = cordic_rotate_base(x_in, y_in, z_in, n, atan_table)
    x = x_in;
    y = y_in;
    z = z_in; % Angolo da ridurre a zero
    
    for i = 1:n
        sigma = sign(z);
        if sigma == 0 && z ~= 0 % Evita sign(0) quando z è molto piccolo
            sigma = sign(z + eps); 
        elseif sigma == 0 % Se z è esattamente 0
            sigma = 1; % Scegli una direzione per continuare le iterazioni
        end
        
        power_of_2 = 2^(-(i-1)); 
        
        x_new = x - sigma * y * power_of_2;
        y_new = y + sigma * x * power_of_2;
        z_new = z - sigma * atan_table(i); 
        
        x = x_new;
        y = y_new;
        z = z_new;
    end
    x_out = x;
    y_out = y;
    z_out = z;
end
% --- CORDIC SIN (modalità rotazione) ---
function sin_val = cordic_sin_function(theta, n, k_factor, atan_table)
    % Normalizza l'angolo a [-pi, pi)
    theta_norm = atan2(sin(theta), cos(theta)); % Usa atan2 di MATLAB per una normalizzazione robusta
    quadrant = 0; % 0: [0, pi/2], 1: (pi/2, pi], 2: (-pi, -pi/2], 3: (-pi/2, 0)
    angle_for_cordic = theta_norm; % L'angolo che useremo per il CORDIC
    if theta_norm >= 0 && theta_norm <= pi/2
        quadrant = 0;
    elseif theta_norm > pi/2 && theta_norm <= pi
        quadrant = 1;
        angle_for_cordic = pi - theta_norm; % Mappa a [0, pi/2]
    elseif theta_norm < 0 && theta_norm >= -pi/2
        quadrant = 3;
        angle_for_cordic = -theta_norm; % Mappa a [0, pi/2]
    elseif theta_norm < -pi/2 && theta_norm > -pi
        quadrant = 2;
        angle_for_cordic = abs(-pi - theta_norm); % Mappa a [0, pi/2]
    end
    
    % Inizializza con x=1, y=0 per calcolare sin/cos.
    % La funzione cordic_rotate_base restituirà x_final e y_final senza K.
    [~, y_cordic_raw, ~] = cordic_rotate_base(1, 0, angle_for_cordic, n, atan_table);
    % Applica il fattore K
    y_cordic_scaled = y_cordic_raw * k_factor;
    % Applica il segno basato sul quadrante originale
    if quadrant == 1 || quadrant == 2 % Secondo o terzo quadrante (sin è negativo nel 3°)
        % Nel secondo quadrante sin è positivo, nel terzo è negativo
        % Ma l'angolo è stato mappato a [0, pi/2].
        % Se quadrant = 1 (originale > pi/2), sin è positivo.
        % Se quadrant = 2 (originale < -pi/2 e > -pi), sin è negativo.
        if theta_norm > 0 % Per il secondo quadrante
            sin_val = y_cordic_scaled;
        else % Per il terzo quadrante
            sin_val = -y_cordic_scaled;
        end
    elseif quadrant == 3 % Quarto quadrante
        sin_val = -y_cordic_scaled;
    else % Primo quadrante
        sin_val = y_cordic_scaled;
    end
    % Gestione dei punti esatti per evitare errori logaritmici su 0
    % Questi dovrebbero già essere gestiti dal max(eps, abs(error))
    if abs(mod(theta, pi)) < 1e-12 || abs(mod(theta, 2*pi)) < 1e-12
        sin_val = 0; % Force 0 for perfect multiples of pi
    end
end
% --- CORDIC ASIN (tramite ATAN) ---
function asin_val = cordic_asin_function(x_input, n, atan_table)
    if abs(x_input) > 1
        asin_val = NaN; % Fuori dominio
        return;
    end
    
    if abs(x_input) == 1 % Casi limite per evitare sqrt(0)
        asin_val = sign(x_input) * pi/2;
        return;
    end
    
    y = x_input;
    x = sqrt(1 - x_input^2); 
    
    % Chiama la funzione base atan per ottenere l'angolo in [-pi, pi]
    % Poi la mappatura a [0, 2pi] avviene a livello di main loop.
    asin_val = cordic_atan_function_base(y, x, n, atan_table);
end
% --- CORDIC ATAN (Mode Vettorizzazione - base per atan2 di MATLAB, range -pi a pi) ---
function angle_z = cordic_atan_function_base(y_input, x_input, n, atan_table)
    x = x_input;
    y = y_input;
    angle_z = 0; 
    % Gestione preliminare dei quadranti (simile a atan2 di MATLAB)
    if x == 0 && y == 0
        angle_z = 0; % O NaN, a seconda della convenzione
        return;
    elseif x == 0 && y > 0
        angle_z = pi/2;
        return;
    elseif x == 0 && y < 0
        angle_z = -pi/2;
        return;
    end
    
    % Ruota il vettore nel semipiano destro (x > 0)
    % Applica la compensazione iniziale dell'angolo
    if x < 0 && y >= 0 % Quadrante II
        angle_z = pi;
        x_temp = abs(x); % Riflessione su Y-axis
        y_temp = -y;
        x = x_temp;
        y = y_temp;
    elseif x < 0 && y < 0 % Quadrante III
        angle_z = -pi;
        x_temp = abs(x); % Riflessione su Y-axis
        y_temp = -y;
        x = x_temp;
        y = y_temp;
    end
    
    % Iterazioni CORDIC
    for i = 1:n
        sigma = -sign(y);
        if sigma == 0 && y ~= 0 
            sigma = sign(y);
        end
        
        power_of_2 = 2^(-(i-1)); 
        
        x_new = x - sigma * y * power_of_2;
        y_new = y + sigma * x * power_of_2;
        
        angle_z = angle_z - sigma * atan_table(i); 
        
        x = x_new;
        y = y_new;
    end
end
% --- Funzione di Wrapper per ATAN per mappare il risultato a [0, 2*pi] ---
function angle_mapped = cordic_atan_function_map_0_2pi(y_input, x_input, n, atan_table)
    % Calcola l'angolo CORDIC nel range standard [-pi, pi]
    angle_raw = cordic_atan_function_base(y_input, x_input, n, atan_table);
    
    % Mappa l'angolo nel range [0, 2*pi]
    if angle_raw < 0
        angle_mapped = angle_raw + 2*pi;
    else
        angle_mapped = angle_raw;
    end
end