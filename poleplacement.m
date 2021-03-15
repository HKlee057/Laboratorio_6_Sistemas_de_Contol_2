% =========================================================================
% IE3041 - ESTABILIZACIÓN DE SISTEMAS LTI
% -------------------------------------------------------------------------
% En este script veremos cómo implementar pole placement para estabilizar 
% el modelo linealizado de un helicóptero.
% Estados:
% x1 - pitch attitude
% x2 - roll attitude
% x3 - roll rate
% x4 - pitch rate
% x5 - yaw rate
% x6 - forward velocity
% x7 - lateral velocity
% x8 - vertical velocity
% =========================================================================
clear all;

%% Matrices continuas del sistema LTI
A1 = [          0                  0                  0   0.99857378005981;
                 0                  0   1.00000000000000  -0.00318221934140;
                 0                  0 -11.57049560546880  -2.54463768005371;
                 0                  0   0.43935656547546  -1.99818229675293;
                 0                  0  -2.04089546203613  -0.45899915695190;
-32.10360717773440                  0  -0.50335502624512   2.29785919189453;
  0.10216116905212  32.05783081054690  -2.34721755981445  -0.50361156463623;
 -1.91097259521484   1.71382904052734  -0.00400543212891  -0.05741119384766];

A2 = [0.05338427424431             0                  0                  0;
  0.05952465534210                  0                  0                  0;
 -0.06360262632370   0.10678052902222  -0.09491866827011   0.00710757449269;
                 0   0.01665188372135   0.01846204698086  -0.00118747074157;
 -0.73502779006958   0.01925575733185  -0.00459562242031   0.00212036073208;
                 0  -0.02121581137180  -0.02116791903973   0.01581159234047;
  0.83494758605957   0.02122657001019  -0.03787973523140   0.00035400385968;
                 0   0.01398963481188  -0.00090675335377  -0.29051351547241];

A = [A1, A2];

B = [             0                  0                  0                  0;
                  0                  0                  0                  0;
   0.12433505058289   0.08278584480286  -2.75247764587402  -0.01788876950741;
  -0.03635892271996   0.47509527206421   0.01429074257612                  0;
   0.30449151992798   0.01495801657438  -0.49651837348938  -0.20674192905426;
   0.28773546218872  -0.54450607299805  -0.01637935638428                  0;
  -0.01907348632812   0.01636743545532  -0.54453611373901   0.23484230041504;
  -4.82063293457031  -0.00038146972656                  0                  0];

C = [  0        0         0         0         0    0.0595   0.05329  -0.9968;
     1.0        0         0         0         0         0         0        0;
       0      1.0         0         0         0         0         0        0;
       0        0         0  -0.05348       1.0         0         0        0;
       0        0       1.0         0         0         0         0        0;
       0        0         0       1.0         0         0         0        0];

%% Parámetros de la simulación
dt = 0.01; % período de muestreo (step size)
t0 = 0;
tf = 10;
K = (tf-t0) / dt;

%% Inicialización y condiciones iniciales
x0 = 0.1 .* ones(8,1); % condiciones iniciales 
u0 = zeros(4,1);
x = x0; % vector de estado
u = u0; % entrada
% Array para almacenar las trayectorias de las variables de estado
X = zeros(numel(x0), K+1); X(:,1) = x;
% Array para almacenar la evolución de las entradas 
U = zeros(numel(u0), K+1); U(:,1) = u; 

%% Discretización del sistema continuo
% Discretización por ZOH
sysd = c2d(ss(A,B,C,0), dt, 'zoh');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;

%% Diseño de control por pole placement
% Seleccionamos nuestros polos favoritos (no se pueden repetir en Matlab):
p = [-1, -1.1, -1.2, -1.3, -1.4, -1.5, -1.6, -1.7]; 
% Usamos la función de pole placement con las matrices del sistema continuo
Kpp = place(A,B,p);  

%% Solución recursiva del sistema dinámico
for k = 1:K
    % Implementación del controlador
    u = -Kpp*x; % retroalimentación de estado
    
    % Se propaga el sistema LTI discreto para aproximar la solución del
    % sistema continuo
    x = Ad*x + Bd*u;
    
    % Se guardan las trayectorias del estado, entrada y salida
    X(:, k+1) = x;
    U(:, k+1) = u;
end

% Graficamos los resultados
t = t0:dt:tf;
figure;
plot(t, X', 'LineWidth', 1);
title('Trayectorias de variables de estado');
xlabel('$t$','Interpreter','latex','FontSize', 16);
ylabel('$\mathbf{x}(t)$', 'Interpreter', 'latex', 'FontSize', 16);
l = legend('$x_1(t)$', '$x_2(t)$', '$x_3(t)$', '$x_4(t)$', '$x_5(t)$', ...
    '$x_6(t)$', '$x_7(t)$', '$x_8(t)$', 'Location', 'southeast');
set(l, 'Interpreter', 'latex', 'FontSize', 12);

figure;
plot(t, U', 'LineWidth', 1);
title('Entradas al sistema con respecto al tiempo');
xlabel('$t$','Interpreter','latex','FontSize', 16);
ylabel('$\mathbf{u}(t)$', 'Interpreter', 'latex', 'FontSize', 16);
l = legend('$u_1(t)$', '$u_2(t)$', '$u_3(t)$', '$u_4(t)$', ...
    'Location', 'southeast');
set(l, 'Interpreter', 'latex', 'FontSize', 12);
