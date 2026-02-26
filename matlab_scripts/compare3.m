clc; clear;
iter = 1:32;
punti_marker = [1, 8, 16, 32];
% === Dati simulati ===
% --- SIN ---
err_sin_board     = 1e-1 * exp(-0.3 * iter);
err_sin_tb        = 8e-2 * exp(-0.3 * iter);
err_sin_matlab    = 2e-16 * ones(1, 32);
var_sin_board     = 5e-3 * exp(-0.25 * iter);
var_sin_tb        = 2e-3 * exp(-0.25 * iter);
var_sin_matlab    = 5e-17 * ones(1, 32);
% --- ASIN ---
err_asin_board    = 2e-1 * exp(-0.28 * iter);
err_asin_tb       = 1.6e-1 * exp(-0.28 * iter);
err_asin_matlab   = 3e-16 * ones(1, 32);
var_asin_board    = 6e-3 * exp(-0.22 * iter);
var_asin_tb       = 3e-3 * exp(-0.22 * iter);
var_asin_matlab   = 8e-17 * ones(1, 32);
% --- ATAN ---
err_atan_board    = 1.5e-1 * exp(-0.26 * iter);
err_atan_tb       = 1.1e-1 * exp(-0.26 * iter);
err_atan_matlab   = 2e-16 * ones(1, 32);
var_atan_board    = 4e-3 * exp(-0.2 * iter);
var_atan_tb       = 2.5e-3 * exp(-0.2 * iter);
var_atan_matlab   = 5e-17 * ones(1, 32);
% === Grafici ===
plot_comparativo('Seno',        iter, ...
    err_sin_board, err_sin_tb, err_sin_matlab, ...
    var_sin_board, var_sin_tb, var_sin_matlab, ...
    'grafico_comparativo_sin_finale');
plot_comparativo('Arcoseno',    iter, ...
    err_asin_board, err_asin_tb, err_asin_matlab, ...
    var_asin_board, var_asin_tb, var_asin_matlab, ...
    'grafico_comparativo_asin_finale');
plot_comparativo('Arcotangente',iter, ...
    err_atan_board, err_atan_tb, err_atan_matlab, ...
    var_atan_board, var_atan_tb, var_atan_matlab, ...
    'grafico_comparativo_atan_finale');
%% === Funzione di plotting ===
function plot_comparativo(nome, iter, ...
    err_board, err_tb, err_matlab, ...
    var_board, var_tb, var_matlab, ...
    filename)
    figure('Color', 'w');
    % Asse sinistro: Errori
    yyaxis left;
    plot(iter, err_board, '-ob', 'LineWidth', 1.8, 'MarkerIndices', [1,8,16,32]); hold on;
    plot(iter, err_tb,    '-sg', 'LineWidth', 1.8, 'MarkerIndices', [1,8,16,32]);
    plot(iter, err_matlab,'-k', 'LineWidth', 1.8); % MATLAB error: black continuous line
    ylabel('Errore Medio', 'FontSize', 12);
    set(gca, 'YScale', 'log', 'YColor', [0 0 1], 'FontSize', 12);  % Asse blu
    ylim([1e-17 1e0]);
    % Asse destro: Varianze
    yyaxis right;
    % Varianza Board: stesso colore dell'errore (blu), tratteggiata
    plot(iter, var_board, '--b', 'LineWidth', 1.8); hold on;
    % Varianza Testbench: stesso colore dell'errore (verde), tratteggiata
    plot(iter, var_tb,    '--g', 'LineWidth', 1.8);
    % Varianza MATLAB: stesso colore dell'errore (nero), tratteggiata
    plot(iter, var_matlab,'--k', 'LineWidth', 1.8);
    ylabel('Varianza', 'FontSize', 12);
    set(gca, 'YScale', 'log', 'YColor', [1 0 0], 'FontSize', 12);  % Asse rosso
    ylim([1e-17 1e0]);
    % Titolo e asse X
    title(['Grafico Comparativo - ', nome], 'FontSize', 14);
    xlabel('Numero di Iterazioni', 'FontSize', 12);
    xlim([1 32]); grid on;
    % Legenda (in area libera fuori dal grafico)
    legend({'Errore Board', 'Errore Testbench', 'Errore MATLAB', ...
            'Varianza Board', 'Varianza Testbench', 'Varianza MATLAB'}, ...
           'Location', 'northoutside', 'NumColumns', 2, 'Box', 'off', 'FontSize', 12); % Legenda fuori in alto, con 2 colonne
    % Salvataggio
    exportgraphics(gcf, [filename, '.png'], 'Resolution', 300);
end