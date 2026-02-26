% Iterazioni da 1 a 32
n_values = 1:32;
% Placeholder dei dati – Sostituisci con dati reali se disponibili
mean_sin  = 10.^(-3 - 0.1*(n_values-1));
var_sin   = mean_sin / 10;
mean_asin = 10.^(-3 - 0.09*(n_values-1));
var_asin  = mean_asin / 10;
mean_atan = 10.^(-3 - 0.095*(n_values-1));
var_atan  = mean_atan / 10;
% Funzioni da plottare
func     = {'SIN','ASIN','ATAN'};
mean_all = {mean_sin, mean_asin, mean_atan};
var_all  = {var_sin,  var_asin,  var_atan};
% Percorso e formato file
folder = 'grafici_salvati';
if ~exist(folder, 'dir')
    mkdir(folder);
end
for k = 1:3
    mean_err = mean_all{k};
    var_err  = var_all{k};
    fig = figure('Visible','on'); hold on;
    % Errore medio (asse sinistro, blu)
    yyaxis left
    h1 = semilogy(n_values, mean_err, '-b', 'LineWidth', 2);
    ax = gca;
    ax.YColor = 'b';
    ylabel('Errore medio (log_{10})', 'FontSize', 12);
    % Marker su 1,8,16,32
    idx_marker = ismember(n_values, [1 8 16 32]);
    plot(n_values(idx_marker), mean_err(idx_marker), 'bo', ...
         'MarkerSize', 8, 'MarkerFaceColor', 'b');
    % Varianza (asse destro, rosso)
    yyaxis right
    h2 = plot(n_values, var_err, '--r', 'LineWidth', 2);
    ax.YColor = 'r';
    ylabel('Varianza', 'FontSize', 12);
    plot(n_values(idx_marker), var_err(idx_marker), 'ro', ...
         'MarkerSize', 8, 'MarkerFaceColor', 'r');
    xlabel('Numero di iterazioni del CORDIC', 'FontSize', 12);
    title(['GRAFICO COMPARATIVO - ' func{k}], 'FontSize', 14);
    grid on;
    xlim([1 32]);
    % Imposta il font size per gli assi
    set(gca, 'FontSize', 12);
    % Legenda interna in alto a destra
    legend([h1 h2], {'Errore medio', 'Varianza'}, ...
           'Location','northeast', 'Box','on', 'FontSize', 12);
    % Salvataggio automatico PNG
    filename = fullfile(folder, ['grafico_comparativo_' lower(func{k}) '.png']);
    saveas(fig, filename);
end