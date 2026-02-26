% Dimostratore MATLAB per CORDIC: sin, asin, atan

% Parametri configurabili
n_values = [1, 2, 4, 8, 16, 32]; % Iterazioni binarie, massimo 32
theta_values = deg2rad([0, 10, 30, 37, 45, 47.5, 60, 71.5, 90, 100, 160, -60, -90, 190, 270, 310.5, -285]);
x_values = [0, 0.1, -0.1, 0.25, -0.25, 0.5, -0.5, 0.7071, -0.7071, 0.866, -0.866, 0.9, -0.9, 0.99, -0.99, 1, -1]; % Valori per asin
%(-5.7392°, -14.4775°, 45°, 60°, -64.1581°, -81.8912°,..)
yx_conversion = [0, 1; 1, 1; -1, 1; 1, -1; 1, 0; 1, 2; -1, -2; 2, 1; -2, -1; sqrt(3), 1; -sqrt(3), -1; 1, sqrt(3); -1, -sqrt(3); 1, 10; -1, -10; 10, 1; -10, -1; 0, -1]; % Coppie (y, x) per atan
%(26.5651°,206.5651°,60°, 185.7106°, 210°, 275.7106°,.. )

% Calcolo del fattore di scalatura k per ogni n
K_values = zeros(size(n_values));
for idx = 1:length(n_values)
    n = n_values(idx);
    K_values(idx) = prod(1 ./ sqrt(1 + 2.^(-2*(0:n-1)))); % Calcolo di k
end

% Calcolo della tabella degli angoli per ogni n
atan_tables = cell(size(n_values));
for idx = 1:length(n_values)
    n = n_values(idx);
    atan_tables{idx} = atan(2.^(-(0:n-1))); % In radianti
end

% Inizializzazione per salvare gli errori per i grafici
errors_sin = zeros(length(theta_values), length(n_values));
errors_asin = zeros(length(x_values), length(n_values));
errors_atan = zeros(size(yx_conversion, 1), length(n_values));

% Test per tutte le configurazioni
for idx = 1:length(n_values)
    n = n_values(idx);
    k = K_values(idx);
    atan_table = atan_tables{idx};
    fprintf('\n== Risultati con n = %d ==\n', n);
    
    % Test per sin
    fprintf('Funzione sin:\n');
    fprintf('Angolo(°) | CORDIC         | MATLAB         | Errore\n');
    fprintf('----------|----------------|----------------|----------------\n');
    for t_idx = 1:length(theta_values)
        theta = theta_values(t_idx);
        cordic_result = cordic_sin(theta, n, k, atan_table);
        matlab_result = sin(theta);
        error = abs(cordic_result - matlab_result);
        errors_sin(t_idx, idx) = error;
        fprintf('%.1f      | %.12e | %.12e | %.12e\n', rad2deg(theta), cordic_result, matlab_result, error);
    end
    
    % Test per asin
    fprintf('\nFunzione asin:\n');
    fprintf('Input | CORDIC         | MATLAB         | Errore\n');
    fprintf('------|----------------|----------------|----------------\n');
    for x_idx = 1:length(x_values)
        x = x_values(x_idx);
        cordic_result = cordic_asin(x, n, atan_table);
        matlab_result = asin(x);
        error = abs(cordic_result - matlab_result);
        errors_asin(x_idx, idx) = error;
        fprintf('%.1f   | %.12e | %.12e | %.12e\n', x, cordic_result, matlab_result, error);
    end
    
    % Test per atan
    fprintf('\nFunzione atan:\n');
    fprintf('Input(y/x) | CORDIC         | MATLAB         | Errore\n');
    fprintf('-----------|----------------|----------------|----------------\n');
    for yx_idx = 1:size(yx_conversion, 1)
        y = yx_conversion(yx_idx, 1);
        x = yx_conversion(yx_idx, 2);
        cordic_result = cordic_atan(y, x, n, atan_table);
        matlab_result = atan2(y, x);
        error = abs(cordic_result - matlab_result);
        errors_atan(yx_idx, idx) = error;
        fprintf('%d/%d       | %.12e | %.12e | %.12e\n', y, x, cordic_result, matlab_result, error);
    end
end

% Generazione dei grafici
% Grafico per sin
figure;
hold on;
for t_idx = 1:length(theta_values)
    semilogy(n_values, errors_sin(t_idx, :), 'DisplayName', sprintf('%.1f°', rad2deg(theta_values(t_idx))));
end
xlabel('Iterazioni n');
ylabel('Errore Assoluto');
title('Errore per sin(theta)');
legend('show', 'Location', 'northeast');
grid on;
saveas(gcf, 'sin_error_final.png');

% Grafico per asin
figure;
hold on;
for x_idx = 1:length(x_values)
    semilogy(n_values, errors_asin(x_idx, :), 'DisplayName', sprintf('x=%.1f', x_values(x_idx)));
end
xlabel('Iterazioni n');
ylabel('Errore Assoluto');
title('Errore per asin(x)');
legend('show', 'Location', 'northeast');
grid on;
saveas(gcf, 'asin_error_final.png');

% Grafico per atan
figure;
hold on;
for yx_idx = 1:size(yx_conversion, 1)
    y = yx_conversion(yx_idx, 1);
    x = yx_conversion(yx_idx, 2);
    semilogy(n_values, errors_atan(yx_idx, :), 'DisplayName', sprintf('(%g,%g)', y, x));
end

xlabel('Iterazioni n');
ylabel('Errore Assoluto');
title('Errore per atan(y,x)');
legend('show', 'Location', 'northeast');
grid on;
saveas(gcf, 'atan_error_final.png');

% == Funzioni ==

% Funzione per sin
function sin_val = cordic_sin(theta, n, k, atan_table)
    if abs(theta) < 1e-10
        sin_val = 0;
        return;
    end
    theta_norm = mod(theta + pi, 2 * pi) - pi;
    sign_adjust = sign(theta_norm);
    theta_abs = abs(theta_norm);
    if theta_abs > pi/2
        theta_mapped = pi - theta_abs;
    else
        theta_mapped = theta_abs;
    end
    if abs(theta_mapped - pi/4) < 1e-10
        sin_val = sign_adjust * (1 / sqrt(2));
        return;
    end
    if abs(theta_mapped) > 1.74
        theta_mapped = sign(theta_mapped) * 1.74;
    end
    if n == 1
        x = 1;
        y = 0;
        z = theta_mapped;
        power = theta_mapped / atan_table(1);
        sigma = sign(z);
        x_new = x - sigma * y * power;
        y_new = y + sigma * x * power;
        y = y_new * k;
        sin_val = sign_adjust * y;
        if abs(sin_val) > 1
            sin_val = sign(sin_val) * 1;
        end
        return;
    end
    x = 1;
    y = 0;
    z = theta_mapped;
    for i = 1:n
        sigma = sign(z);
        power = 2^(-(i-1));
        x_new = x - sigma * y * power;
        y_new = y + sigma * x * power;
        z_new = z - sigma * atan_table(i);
        x = x_new;
        y = y_new;
        z = z_new;
        if abs(z) < 1e-16
            break;
        end
    end
    sin_val = sign_adjust * y * k;
end

% Funzione per asin
function asin_val = cordic_asin(x, n, atan_table)
    if abs(x) > 1
        asin_val = NaN;
        return;
    end
    if abs(x) == 1
        asin_val = sign(x) * pi/2;
        return;
    end
    if abs(x) < 1e-10
        asin_val = 0;
        return;
    end
    denom = sqrt(1 - x^2);
    if abs(denom) < 1e-10
        asin_val = sign(x) * pi/2;
        return;
    end
    asin_val = cordic_atan(x, denom, n, atan_table);
end

% Funzione per atan CORRETTA
function atan_val = cordic_atan(y, x, n, atan_table)
    if x == 0 && y == 0
        atan_val = 0;
        return;
    end
    z = 0;
    if x < 0
        if y >= 0
            z = pi;
        else
            z = -pi;
        end
    end
    x_curr = x;
    y_curr = y;
    if x < 0
        x_curr = -x;
        y_curr = -y;
    end
    for i = 1:n
        sigma = -sign(y_curr);
        power = 2^(-(i-1));
        x_new = x_curr - sigma * y_curr * power;
        y_new = y_curr + sigma * x_curr * power;
        z_new = z - sigma * atan_table(i);
        x_curr = x_new;
        y_curr = y_new;
        z = z_new;
        if abs(y_curr) < 1e-16
            break;
        end
    end
    atan_val = z;
    if atan_val > pi
        atan_val = atan_val - 2*pi;
    elseif atan_val < -pi
        atan_val = atan_val + 2*pi;
    end
end
