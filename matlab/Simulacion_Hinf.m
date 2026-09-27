%% ANALISIS COMPLETO DEL CONTROLADOR H-INFINITO — AEROPENDULO
%  Genera CADA grafica como una imagen independiente.
%
%  Ejecutar:  analisis_hinf
%  Requiere:  Control System Toolbox
%
%  Salida: 16 PNG (300 dpi) + 16 PDF vectoriales en:
%          C:\Users\nicao\Downloads\Hinf_imagenes

clear; clc; close all;

%% ==================================================================
%  0. CARPETA DE SALIDA Y OPCIONES DE EXPORTACION
%  ==================================================================
OUT_DIR    = 'C:\Users\nicao\Downloads\Hinf_imagenes';
EXPORT_PNG = true;    % PNG a 300 dpi (para diapositivas)
EXPORT_PDF = true;    % PDF vectorial (para el informe)
DPI        = 300;
FIG_W      = 900;     % ancho en pixeles
FIG_H      = 480;     % alto en pixeles

if ~exist(OUT_DIR, 'dir')
    mkdir(OUT_DIR);
    fprintf('Carpeta creada: %s\n\n', OUT_DIR);
else
    fprintf('Guardando en: %s\n\n', OUT_DIR);
end

nfig = 0;

%% ==================================================================
%  1. CONFIGURACION
%  ==================================================================
cfg.Ts_control   = 0.010;   % periodo del lazo [s]
cfg.tau_motor    = 0.05;    % constante de tiempo motor+helice [s]
cfg.ruido_deg    = 1.0;     % ruido del sensor [grados]
cfg.usar_u0eq    = false;   % el integrador encuentra el equilibrio solo
cfg.u0_eq        = 35.1444323672;

cfg.theta_ini    = 0;       % angulo inicial [grados]
cfg.ref_final    = 70;      % referencia [grados]
cfg.t_arranque   = 1.0;     % instante del escalon [s]
cfg.T_sim        = 15.0;    % duracion [s]

cfg.usar_rampa   = false;
cfg.slew_dps     = 15;

cfg.limite_fisico = 130;    % tope mecanico [grados]

theta_op = 70;

%% ==================================================================
%  2. PLANTA
%  ==================================================================
P.m = 0.080;      P.g = 9.81;       P.d = 0.05;
P.J = 4.2860e-4;  P.b = 8.3032e-4;  P.K = 1.0492e-3;

th0 = deg2rad(theta_op);
A1 = P.b / P.J;
A0 = (P.m*P.g*P.d*cos(th0)) / P.J;
B0 = (P.K/P.J) * (180/pi);

G  = tf(B0, [1 A1 A0]);
Gd = c2d(G, cfg.Ts_control, 'zoh');

%% ==================================================================
%  3. CONTROLADOR
%  ==================================================================
num_z = [ 1.15819064107, -0.463870070534, -1.80823927917, ...
          0.469530856268, 0.655709423838];
den_z = [ 1, -1.54880027756, 0.124342533919, ...
          0.750269557161, -0.325809250827];

Kz = tf(num_z, den_z, cfg.Ts_control);

%% ==================================================================
%  4. FUNCIONES DE LAZO
%  ==================================================================
L  = Kz * Gd;
S  = feedback(1, L);
T  = feedback(L, 1);
KS = feedback(Kz, Gd);
SG = feedback(Gd, Kz);

%% ==================================================================
%  5. INFORME EN CONSOLA
%  ==================================================================
linea = @() fprintf('%s\n', repmat('=', 1, 68));

linea();
fprintf(' PLANTA LINEALIZADA EN %g GRADOS\n', theta_op);
linea();
fprintf('  G(s) = %.3f / (s^2 + %.4f s + %.4f)\n', B0, A1, A0);
fprintf('  Frecuencia natural :  %.3f rad/s\n', sqrt(A0));
fprintf('  Amortiguamiento    :  %.4f\n', A1/(2*sqrt(A0)));
fprintf('  PWM de equilibrio  :  %.4f %%\n\n', P.m*P.g*P.d*sin(th0)/P.K);

linea();
fprintf(' CONTROLADOR H-INFINITO\n');
linea();
fprintf('  Orden        :  %d\n', length(den_z)-1);
fprintf('  Ts           :  %g s  (%g Hz)\n', cfg.Ts_control, 1/cfg.Ts_control);
fprintf('  Ganancia DC  :  %.1f\n', dcgain(Kz));
fprintf('  Polos:\n');
pk = pole(Kz);
for i = 1:numel(pk)
    marca = '';
    if abs(abs(pk(i))-1) < 1e-4, marca = '   <-- integrador'; end
    fprintf('     %+8.5f %+8.5fi   |z| = %.5f%s\n', ...
        real(pk(i)), imag(pk(i)), abs(pk(i)), marca);
end
fprintf('  Ceros:\n');
zk = zero(Kz);
for i = 1:numel(zk)
    fprintf('     %+8.5f %+8.5fi   |z| = %.5f\n', ...
        real(zk(i)), imag(zk(i)), abs(zk(i)));
end
fprintf('\n');

[Gm, Pm, Wcg, Wcp] = margin(L);

linea();
fprintf(' MARGENES DE ESTABILIDAD\n');
linea();
fprintf('  Margen de ganancia :  %8.2f dB   (en %.3f rad/s)\n', 20*log10(Gm), Wcg);
fprintf('  Margen de fase     :  %8.2f grados (en %.3f rad/s)\n', Pm, Wcp);
fprintf('  Retardo tolerable  :  %8.1f ms\n', 1000*deg2rad(Pm)/Wcp);
fprintf('  Estable en lazo cerrado: %s\n\n', string(all(abs(pole(T)) < 1)));

nS  = norm(S,  Inf);
nT  = norm(T,  Inf);
nKS = norm(KS, Inf);

linea();
fprintf(' NORMAS H-INFINITO\n');
linea();
fprintf('  ||S||inf   =  %8.4f\n', nS);
fprintf('  ||T||inf   =  %8.4f\n', nT);
fprintf('  ||KS||inf  =  %8.4f\n', nKS);
fprintf('  Margen de modulo = 1/||S|| = %.4f\n', 1/nS);
try
    bw = bandwidth(T);
    fprintf('  Ancho de banda   =  %.2f rad/s  (%.2f Hz)\n', bw, bw/(2*pi));
catch
    bw = NaN;
end
fprintf('\n');

Slin = stepinfo(T);

linea();
fprintf(' RESPUESTA AL ESCALON (modelo lineal)\n');
linea();
fprintf('  Tiempo de subida    (10-90%%) :  %7.4f s\n', Slin.RiseTime);
fprintf('  Tiempo de pico               :  %7.4f s\n', Slin.PeakTime);
fprintf('  Sobrepaso                    :  %7.2f %%\n', Slin.Overshoot);
fprintf('  Submontaje                   :  %7.2f %%\n', Slin.Undershoot);
fprintf('  Tiempo de estab. (2%%)        :  %7.4f s\n', Slin.SettlingTime);
fprintf('  Valor final                  :  %7.4f\n', dcgain(T));
fprintf('  Error estacionario           :  %7.4f\n\n', 1 - dcgain(T));

%% ==================================================================
%  6. SIMULACION NO LINEAL
%  ==================================================================
[t, theta, pwm, ref, tope] = correr(cfg, P, num_z, den_z);

theta_fin = mean(theta(t > t(end)-2));
pico      = max(theta);
sobrep    = (pico - cfg.ref_final)/cfg.ref_final * 100;
err_ee    = cfg.ref_final - theta_fin;

banda = 0.02*cfg.ref_final;
idx = find(abs(theta - cfg.ref_final) > banda, 1, 'last');
if isempty(idx), t_est = 0; else, t_est = t(min(idx+1,numel(t))) - cfg.t_arranque; end

i10 = find(theta >= 0.1*cfg.ref_final, 1);
i90 = find(theta >= 0.9*cfg.ref_final, 1);
if ~isempty(i10) && ~isempty(i90), t_subida = t(i90)-t(i10); else, t_subida = NaN; end

linea();
fprintf(' SIMULACION NO LINEAL (%g -> %g grados)\n', cfg.theta_ini, cfg.ref_final);
linea();
if tope
    fprintf('  !! GOLPEA EL TOPE MECANICO de %g grados\n', cfg.limite_fisico);
end
fprintf('  Angulo final              :  %7.2f grados\n', theta_fin);
fprintf('  Pico maximo               :  %7.2f grados\n', pico);
fprintf('  Sobrepaso                 :  %7.2f %%\n', sobrep);
fprintf('  Tiempo de subida (10-90%%) :  %7.3f s\n', t_subida);
fprintf('  Tiempo de estab. (2%%)     :  %7.3f s\n', t_est);
fprintf('  Error estacionario        :  %7.3f grados\n', err_ee);
fprintf('  PWM medio en regimen      :  %7.2f %%\n', mean(pwm(t > t(end)-2)));
fprintf('  Rizado del PWM (desv.)    :  %7.2f %%\n', std(pwm(t > t(end)-2)));
fprintf('  PWM saturado              :  %7.2f %% del tiempo\n\n', ...
    100*mean(pwm >= 99.9 | pwm <= 0.1));

%% ==================================================================
%  7. BARRIDOS DE ROBUSTEZ
%  ==================================================================
linea();
fprintf(' BARRIDO 1 — PERIODO DE MUESTREO REAL\n');
linea();
fprintf('  %-12s %-14s %-14s %s\n','Ts [ms]','Pico [grados]','Sobrepaso [%]','Estado');
Ts_list = [10 15 20 25 30 40]/1000;
pico_Ts = zeros(size(Ts_list));
for i = 1:numel(Ts_list)
    c = cfg; c.Ts_control = Ts_list(i);
    [~, th, ~, ~, tp] = correr(c, P, num_z, den_z);
    pico_Ts(i) = max(abs(th));
    if tp,                      est = 'GOLPEA TOPE';
    elseif pico_Ts(i) > 1e4,    est = 'DIVERGE';
    else,                       est = 'ok';
    end
    fprintf('  %-12.0f %-14.1f %-14.1f %s\n', Ts_list(i)*1000, pico_Ts(i), ...
        (pico_Ts(i)-cfg.ref_final)/cfg.ref_final*100, est);
end

fprintf('\n');
linea();
fprintf(' BARRIDO 2 — RETARDO DEL ACTUADOR\n');
linea();
fprintf('  %-14s %-14s %s\n','tau [ms]','Pico [grados]','Estado');
tau_list = [0 20 50 80 100 150 200]/1000;
pico_tau = zeros(size(tau_list));
for i = 1:numel(tau_list)
    c = cfg; c.tau_motor = tau_list(i);
    [~, th, ~, ~, tp] = correr(c, P, num_z, den_z);
    pico_tau(i) = max(abs(th));
    if tp, est = 'GOLPEA TOPE'; else, est = 'ok'; end
    fprintf('  %-14.0f %-14.1f %s\n', tau_list(i)*1000, pico_tau(i), est);
end

fprintf('\n');
linea();
fprintf(' BARRIDO 3 — VARIACION PARAMETRICA (+/-20%% en J, b, K)\n');
linea();
variaciones = [0.8 1.0 1.2];
estables = 0; total = 0; peor_pico = 0;
for fJ = variaciones
    for fb = variaciones
        for fK = variaciones
            Pv = P; Pv.J = P.J*fJ; Pv.b = P.b*fb; Pv.K = P.K*fK;
            [~, th, ~, ~, tp] = correr(cfg, Pv, num_z, den_z);
            total = total + 1;
            peor_pico = max(peor_pico, max(abs(th)));
            if ~tp && isfinite(th(end)) && abs(th(end)-cfg.ref_final) < 10
                estables = estables + 1;
            end
        end
    end
end
fprintf('  Casos estables : %d / %d\n', estables, total);
fprintf('  Peor pico      : %.1f grados\n\n', peor_pico);

%% ==================================================================
%  8. GRAFICAS — UNA IMAGEN POR CADA UNA
%  ==================================================================
az  = [0.15 0.35 0.65];
naj = [0.75 0.35 0.15];
vrd = [0.15 0.55 0.35];
mor = [0.45 0.25 0.60];

linea();
fprintf(' EXPORTANDO IMAGENES\n');
linea();

% ---------- 01: angulo vs referencia ----------
f = nuevaFig(FIG_W, FIG_H);
plot(t, ref, '--', 'LineWidth', 1.4, 'Color', [.55 .55 .55]); hold on;
plot(t, theta, 'LineWidth', 1.8, 'Color', az);
yline(cfg.limite_fisico, ':r', 'tope mecanico', 'LineWidth', 1.2, 'HandleVisibility','off');
yline(cfg.ref_final*1.02, ':', 'Color', [.7 .7 .7], 'HandleVisibility','off');
yline(cfg.ref_final*0.98, ':', 'Color', [.7 .7 .7], 'HandleVisibility','off');
grid on; box on;
xlabel('Tiempo [s]'); ylabel('Angulo [grados]');
legend('Referencia','Salida','Location','southeast');
title(sprintf('Respuesta del lazo cerrado no lineal   (Ts=%g ms, \\tau_{motor}=%g ms, ruido=%.1f\\circ)', ...
    cfg.Ts_control*1000, cfg.tau_motor*1000, cfg.ruido_deg));
nfig = guardar(f, '01_respuesta_angulo', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 02: error de seguimiento ----------
f = nuevaFig(FIG_W, FIG_H);
plot(t, ref - theta, 'LineWidth', 1.5, 'Color', vrd); hold on;
yline(0, ':k', 'LineWidth', 1.0);
grid on; box on;
xlabel('Tiempo [s]'); ylabel('Error [grados]');
title('Error de seguimiento   e = referencia - angulo medido');
nfig = guardar(f, '02_error_seguimiento', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 03: senal de control ----------
f = nuevaFig(FIG_W, FIG_H);
plot(t, pwm, 'LineWidth', 1.4, 'Color', naj); hold on;
yline(100, ':k', 'LineWidth', 1.0); yline(0, ':k', 'LineWidth', 1.0);
grid on; box on; ylim([-5 105]);
xlabel('Tiempo [s]'); ylabel('PWM [%]');
title(sprintf('Senal de control   (medio en regimen: %.1f%%,  rizado: %.2f%%)', ...
    mean(pwm(t > t(end)-2)), std(pwm(t > t(end)-2))));
nfig = guardar(f, '03_senal_control', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 04: escalon lineal ----------
f = nuevaFig(FIG_W, FIG_H);
step(T, 8); grid on; box on;
title(sprintf('Respuesta al escalon (modelo lineal)   M_p=%.1f%%,  t_s=%.2f s', ...
    Slin.Overshoot, Slin.SettlingTime));
nfig = guardar(f, '04_escalon_lineal', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 05: polos y ceros del lazo cerrado ----------
f = nuevaFig(FIG_W, FIG_H);
pzmap(T); grid on; box on;
title('Polos y ceros del lazo cerrado (plano z)');
nfig = guardar(f, '05_polos_ceros_lazo_cerrado', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 06: polos y ceros del controlador ----------
f = nuevaFig(FIG_W, FIG_H);
pzmap(Kz); grid on; box on; hold on;
th_u = linspace(0, 2*pi, 300);
plot(cos(th_u), sin(th_u), '--', 'Color', [.6 .6 .6], 'LineWidth', 1.0);
title(sprintf('Polos y ceros del controlador K(z)   (polo en z=%.6f = integrador)', ...
    max(real(pk))));
nfig = guardar(f, '06_polos_ceros_controlador', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 07: rechazo a perturbacion ----------
f = nuevaFig(FIG_W, FIG_H);
step(SG, 8); grid on; box on;
ylabel('Angulo [grados]');
title('Rechazo a perturbacion en la entrada  (S G)');
nfig = guardar(f, '07_perturbacion_entrada', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 08: respuesta al impulso ----------
f = nuevaFig(FIG_W, FIG_H);
impulse(T, 4); grid on; box on;
title('Respuesta al impulso del lazo cerrado');
nfig = guardar(f, '08_respuesta_impulso', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 09: Bode con margenes ----------
f = nuevaFig(FIG_W, 620);
margin(L); grid on;
title(sprintf('Lazo abierto L(z)=K(z)G(z)   MF=%.1f\\circ,  MG=%.1f dB,  \\omega_c=%.2f rad/s', ...
    Pm, 20*log10(Gm), Wcp));
nfig = guardar(f, '09_bode_margenes', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 10: Nyquist ----------
f = nuevaFig(FIG_W, FIG_H);
nyquist(L); grid on; box on; hold on;
th_c = linspace(0, 2*pi, 300);
plot(-1 + (1/nS)*cos(th_c), (1/nS)*sin(th_c), '--r', 'LineWidth', 1.4);
plot(-1, 0, 'rx', 'MarkerSize', 11, 'LineWidth', 2);
title(sprintf('Diagrama de Nyquist   distancia minima a -1 = 1/||S|| = %.3f', 1/nS));
nfig = guardar(f, '10_nyquist', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 11: sensibilidad S y T ----------
f = nuevaFig(FIG_W, FIG_H);
bodemag(S, T); grid on; box on;
legend('S (sensibilidad)','T (complementaria)','Location','best');
title(sprintf('Funciones de sensibilidad   ||S||_\\infty=%.3f,  ||T||_\\infty=%.3f', nS, nT));
nfig = guardar(f, '11_sensibilidad_S_T', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 12: esfuerzo de control ----------
f = nuevaFig(FIG_W, FIG_H);
bodemag(KS, Kz); grid on; box on;
legend('KS (esfuerzo de control)','K (controlador)','Location','best');
title(sprintf('Esfuerzo de control   ||KS||_\\infty=%.3f', nKS));
nfig = guardar(f, '12_esfuerzo_control_KS', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 13: barrido del periodo de muestreo ----------
f = nuevaFig(FIG_W, FIG_H);
bar(Ts_list*1000, min(pico_Ts, 200), 0.6, 'FaceColor', az); hold on;
yline(cfg.limite_fisico, '--r', 'tope mecanico', 'LineWidth', 1.4);
yline(cfg.ref_final, ':k', 'referencia', 'LineWidth', 1.2);
for i = 1:numel(Ts_list)
    text(Ts_list(i)*1000, min(pico_Ts(i),200)+5, sprintf('%.0f', pico_Ts(i)), ...
        'HorizontalAlignment','center','FontSize',9);
end
grid on; box on; ylim([0 215]);
xlabel('Periodo de muestreo real [ms]'); ylabel('Pico maximo [grados]');
title('Sensibilidad al periodo de muestreo del lazo');
nfig = guardar(f, '13_barrido_muestreo', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 14: barrido del retardo del actuador ----------
f = nuevaFig(FIG_W, FIG_H);
bar(tau_list*1000, min(pico_tau, 200), 0.6, 'FaceColor', naj); hold on;
yline(cfg.limite_fisico, '--r', 'tope mecanico', 'LineWidth', 1.4);
yline(cfg.ref_final, ':k', 'referencia', 'LineWidth', 1.2);
for i = 1:numel(tau_list)
    text(tau_list(i)*1000, min(pico_tau(i),200)+5, sprintf('%.0f', pico_tau(i)), ...
        'HorizontalAlignment','center','FontSize',9);
end
grid on; box on; ylim([0 215]);
xlabel('Constante de tiempo del actuador [ms]'); ylabel('Pico maximo [grados]');
title('Sensibilidad al retardo del motor y la helice');
nfig = guardar(f, '14_barrido_retardo_actuador', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 15: robustez parametrica ----------
f = nuevaFig(700, FIG_H);
bar(1, 100*estables/total, 0.5, 'FaceColor', mor); hold on;
text(1, 100*estables/total + 4, sprintf('%d / %d', estables, total), ...
    'HorizontalAlignment','center','FontSize',13,'FontWeight','bold');
ylim([0 115]); xlim([0.4 1.6]);
set(gca,'XTick',1,'XTickLabel',{'J, b, K variando \pm20%'});
grid on; box on;
ylabel('Casos estables [%]');
title(sprintf('Robustez parametrica   (peor pico: %.1f grados)', peor_pico));
nfig = guardar(f, '15_robustez_parametrica', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

% ---------- 16: panel resumen ----------
f = nuevaFig(FIG_W, 720);
subplot(3,1,1);
plot(t, ref, '--', 'LineWidth', 1.3, 'Color', [.55 .55 .55]); hold on;
plot(t, theta, 'LineWidth', 1.6, 'Color', az);
yline(cfg.limite_fisico, ':r', 'LineWidth', 1.1, 'HandleVisibility','off');
grid on; box on; ylabel('Angulo [\circ]');
legend('Referencia','Salida','Location','southeast');
title('Resumen del lazo cerrado no lineal');
subplot(3,1,2);
plot(t, ref - theta, 'LineWidth', 1.3, 'Color', vrd); hold on; yline(0, ':k');
grid on; box on; ylabel('Error [\circ]');
subplot(3,1,3);
plot(t, pwm, 'LineWidth', 1.2, 'Color', naj); hold on;
yline(100, ':k'); yline(0, ':k');
grid on; box on; ylabel('PWM [%]'); xlabel('Tiempo [s]'); ylim([-5 105]);
nfig = guardar(f, '16_panel_resumen', OUT_DIR, EXPORT_PNG, EXPORT_PDF, DPI, nfig);

%% ==================================================================
%  9. TABLA RESUMEN
%  ==================================================================
fprintf('\n');
linea();
fprintf(' RESUMEN PARA EL INFORME\n');
linea();
fprintf('  %-34s %s\n', 'Criterio', 'Valor');
fprintf('  %s\n', repmat('-', 1, 56));
fprintf('  %-34s %.2f dB\n',      'Margen de ganancia',        20*log10(Gm));
fprintf('  %-34s %.2f grados\n',  'Margen de fase',            Pm);
fprintf('  %-34s %.1f ms\n',      'Retardo tolerable',         1000*deg2rad(Pm)/Wcp);
fprintf('  %-34s %.4f\n',         '||S||inf',                  nS);
fprintf('  %-34s %.4f\n',         '||T||inf',                  nT);
fprintf('  %-34s %.4f\n',         '||KS||inf',                 nKS);
if ~isnan(bw)
    fprintf('  %-34s %.2f rad/s\n', 'Ancho de banda',           bw);
end
fprintf('  %-34s %.2f %%\n',      'Sobrepaso (lineal)',        Slin.Overshoot);
fprintf('  %-34s %.3f s\n',       'Tiempo de estab. (lineal)', Slin.SettlingTime);
fprintf('  %-34s %.2f %%\n',      'Sobrepaso (no lineal)',     sobrep);
fprintf('  %-34s %.3f s\n',       'Tiempo de estab. (no lin.)',t_est);
fprintf('  %-34s %.3f grados\n',  'Error estacionario',        err_ee);
fprintf('  %-34s %d/%d\n',        'Robustez parametrica',      estables, total);
linea();
fprintf('\n  %d figuras exportadas en:\n  %s\n\n', nfig, OUT_DIR);

%% ==================================================================
%  FUNCIONES AUXILIARES
%  ==================================================================
function f = nuevaFig(w, h)
    f = figure('Color','w','Position',[80 80 w h]);
    set(f, 'DefaultAxesFontSize', 11);
end

function n = guardar(f, nombre, dir_out, do_png, do_pdf, dpi, n)
    drawnow;
    if do_png
        ruta = fullfile(dir_out, [nombre '.png']);
        try
            exportgraphics(f, ruta, 'Resolution', dpi, 'BackgroundColor', 'white');
        catch
            print(f, ruta, '-dpng', sprintf('-r%d', dpi));
        end
        fprintf('  %s.png\n', nombre);
    end
    if do_pdf
        ruta = fullfile(dir_out, [nombre '.pdf']);
        try
            exportgraphics(f, ruta, 'ContentType', 'vector', 'BackgroundColor', 'white');
        catch
            print(f, ruta, '-dpdf', '-bestfit');
        end
    end
    n = n + 1;
end

function [t, theta_deg, pwm, ref, golpeo_tope] = correr(cfg, P, num_z, den_z)

    Ts = cfg.Ts_control;
    n  = round(cfg.T_sim / Ts);
    t  = (0:n-1)' * Ts;

    theta = deg2rad(cfg.theta_ini);
    omega = 0;  empuje = 0;

    e_hist = zeros(1,5);  u_hist = zeros(1,4);
    theta_deg = zeros(n,1);  pwm = zeros(n,1);  ref = zeros(n,1);
    golpeo_tope = false;

    if cfg.usar_u0eq, u_eq = cfg.u0_eq; else, u_eq = 0; end
    rng(0);

    for k = 1:n
        tk = t(k);

        if tk < cfg.t_arranque
            r = cfg.theta_ini;
        elseif cfg.usar_rampa
            r = min(cfg.ref_final, cfg.theta_ini + (tk-cfg.t_arranque)*cfg.slew_dps);
        else
            r = cfg.ref_final;
        end
        ref(k) = r;

        y = rad2deg(theta) + cfg.ruido_deg*randn;
        e = r - y;
        e_hist = [e, e_hist(1:4)];

        du = num_z*e_hist' - den_z(2:5)*u_hist';
        u_hist = [du, u_hist(1:3)];

        u = min(100, max(0, u_eq + du));
        pwm(k) = u;

        if cfg.tau_motor > 0
            empuje = empuje + (u - empuje)*(Ts/cfg.tau_motor);
        else
            empuje = u;
        end

        dom   = (P.K*empuje - P.b*omega - P.m*P.g*P.d*sin(theta)) / P.J;
        omega = omega + dom*Ts;
        theta = theta + omega*Ts;
        theta_deg(k) = rad2deg(theta);

        if abs(theta_deg(k)) >= cfg.limite_fisico, golpeo_tope = true; end
        if ~isfinite(theta_deg(k)) || abs(theta_deg(k)) > 1e5
            theta_deg(k:end) = theta_deg(k);
            pwm(k:end) = u;  ref(k:end) = r;
            return;
        end
    end
end