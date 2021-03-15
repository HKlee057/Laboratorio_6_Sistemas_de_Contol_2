%% Laboratorio 6
%% Datos
% Hector Alejandro Klée González - 17118
% Juan Diego Castillo Amaya - 17074
% Sección 11 
% Sistemas de Control 2
%% Primera Parte - Diseño y Simulación
%% Inciso 1
% Definición de varaibles
r1 = 1e3;
r2 = 10e3;
r3 = r2;
c1 = 1e-6;
c2 = 0.1e-6;
c3 = 10e-6;
% Recuperación de conjunto de varaibles 
A = [-((r2+r1)/(r1*r2*c1)),1/(r2*c1),-1/(r2*c1);...
    0,0,-1/(r3*c2);...
    -1/(r2*c3),1/(r2*c3),-(r3+r2)/(r2*r3*c3)];
B = [1/(r1*c1);0;0];
C = [-0 1 0];
% Definición de varaible tipo ss
sys = ss(A, B, C, 0);
%% Inciso 2
n = length (A);
Gamma = ctrb(A, B);
rankVal = rank(Gamma);
if (rankVal == n)
    fprintf ('El el sistema es completamente controlable. \n');
else
    fprintf ('El el sistema no es completamente controlable. \n');
end
%% Inciso 3
s = tf("s");
G = C * (inv((s * eye(3)) - A)) * B;
OG_poles = pole(G);
% Vector de polos cercano a los polos originales
% p = [-1.1001e3 (-10 + 0.0948i) (-10 - 0.0948i)];
p = [-1100.1 (-100 + 150i) (-100 - 150i)]; 
%% Inciso 4
K_pp = place(A, B, p); 
%% Inciso 5
A_cl = A - (B * K_pp);
% Definición de nueva varaible tipo ss
sys_cl = ss(A_cl, B, C, 0);
%% Inciso 6
[u,t] = gensig("square", 1/0.5 , 4, 0.0001);
u = u + 1;
%% Inciso 7
% Definición de vectores
[y1,t,X1]=lsim(sys,u,t);
[y2,t,X2]=lsim(sys_cl,u,t);
% Gráficas
figure(1); 
clf;
hold on;
subplot(2,1,1);
plot(t,y1);
title("Gráfica de Sistema Original");
xlabel('t');
ylabel('y Original');
subplot(2,1,2);
plot(t,y2);
title("Gráfica de Sistema en Lazo Cerrado");
xlabel('t');
ylabel('y Modificado')
suptitle("Salidas de Sistema");
hold off;
%% Inciso 9
% Factor de Escala
Nbar = rscale(sys, K_pp);
[y3,t,X3]=lsim(sys_cl, Nbar*u, t);
% Gráficas
figure(2); 
clf;
hold on;
subplot(2,1,1);
plot(t,y2);
title("Gráfica SIN ESCALAR");
xlabel('t');
ylabel('y');
subplot(2,1,2);
plot(t,y3);
title("Gráfica ESCALADA");
xlabel('t');
ylabel('y')
suptitle("Comparación de Salidas");
hold off;
% Verificación
linearSystemAnalyzer(Nbar * sys_cl);