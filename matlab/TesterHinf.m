% =========================================================================
% ANÁLISIS DE LA PLANTA Y CONTROLADOR PID DEL INFORME EN MATLAB
% =========================================================================
clc; clear; close all;

%% 1. DEFINICIÓN DE LA PLANTA G(s) (Grados / %PWM)
% G(s) = 140.3 / (s^2 + 1.937*s + 31.32)
num_G = [140.3];
den_G = [1, 1.937, 31.32];
G = tf(num_G, den_G);

disp('--- Función de Transferencia de la Planta G(s) ---');
G

%% 2. DEFINICIÓN DEL CONTROLADOR PID C(s)
% Ganancias del informe: Kp = 0.28, Ki = 1.0, Kd = 0.20, N = 100
Kp = 0.28; 
Ki = 1.0; 
Kd = 0.20; 
N = 100;

s = tf('s');
C = Kp + Ki/s + (Kd*N*s)/(s + N);

disp('--- Función de Transferencia del Controlador PID C(s) ---');
C = minreal(C)

%% 3. LAZO ABIERTO L(s) Y MÁRGENES DE ESTABILIDAD
L = C * G;

% Graficar el diagrama de Bode de Lazo Abierto con márgenes de Fase y Ganancia
figure('Name', 'Diagrama de Bode de Lazo Abierto L(s)');
margin(L);
grid on;

%% 4. LAZO CERRADO T(s) (Respuesta del sistema)
T = feedback(L, 1);

% Graficar la respuesta al escalón en lazo cerrado
figure('Name', 'Respuesta al Escalón en Lazo Cerrado');
step(T);
title('Respuesta Temporal del Ángulo \theta(t) a un Escalón de Referencia');
ylabel('Ángulo [grados]');
xlabel('Tiempo [segundos]');
grid on;