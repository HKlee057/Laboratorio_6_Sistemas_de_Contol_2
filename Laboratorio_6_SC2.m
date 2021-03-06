%% Laboratorio 6
%% Datos
% Hector Alejandro Kl?e Gonz?lez - 17118
% Juan Diego Castillo Amaya - 17074
% Secci?n 11 
% Sistemas de Control 2
%% Primera Parte - Dise?o y Simulaci?n
load Variables_de_Estado.mat linsys1
%% Inciso 1
% Definici?n de varaibles
r1 = 1e3;
r2 = 10e3;
r3 = r2;
c1 = 1e-6;
c2 = 0.1e-6;
c3 = 10e-6;
% Recuperaci?n de conjunto de varaibles 
A = [-((r2+r1)/(r1*r2*c1)),1/(r2*c1),-1/(r2*c1);...
    0,0,-1/(r3*c2);...
    -1/(r2*c3),1/(r2*c3),-(r3+r2)/(r2*r3*c3)];
B = [1/(r1*c1);0;0];
C = [-0 1 0];
D = 0;
% Definici?n de varaible tipo ss
sys = ss(A, B, C, D);
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
% Definici?n de nueva varaible tipo ss
sys_cl = ss(A_cl, B, C, D);
%% Inciso 6
[u,t] = gensig("square", 1/0.5 , 4, 0.0001);
u = u + 1;
%% Inciso 7
% Definici?n de vectores
[y1,t,X1]=lsim(sys,u,t);
[y2,t,X2]=lsim(sys_cl,u,t);
% Gr?ficas
figure(1); 
clf;
hold on;
subplot(2,1,1);
plot(t,y1);
title("Gr?fica de Sistema Original");
xlabel('t');
ylabel('y Original');
subplot(2,1,2);
plot(t,y2);
title("Gr?fica de Sistema en Lazo Cerrado");
xlabel('t');
ylabel('y Modificado')
suptitle("Salidas de Sistema");
hold off;
%% Inciso 9
% Factor de Escala
Nbar = rscale(sys, K_pp);
[y3,t,X3]=lsim(sys_cl, Nbar*u, t);
% Gr?ficas
figure(2); 
clf;
hold on;
subplot(2,1,1);
plot(t,y2);
title("Gr?fica SIN ESCALAR");
xlabel('t');
ylabel('y');
subplot(2,1,2);
plot(t,y3);
title("Gr?fica ESCALADA");
xlabel('t');
ylabel('y')
suptitle("Comparaci?n de Salidas");
hold off;
% Verificaci?n
linearSystemAnalyzer(Nbar * sys_cl);
%% Inciso 11
sys1=ss(linsys1.A,linsys1.B,linsys1.C,linsys1.D);
%[y4,t,X4]=lsim(sys,u,t);
% figure (8); clf;
% hold on;
% subplot(3,1,1);
% plot(t,X4(:,1));
% title("Gr?fica de C1");
% xlabel('t');
% ylabel('Vc1');
% subplot(3,1,2);
% plot(t,X4(:,2));
% title("Gr?fica de C2");
% xlabel('t');
% ylabel('Vc2')
% subplot(3,1,3);
% plot(t,X4(:,3));
% title("Gr?fica de C3");
% suptitle("Se?al Cuadrada de 5V a 0.5Hz, Variables Linsys");
% xlabel('t');
% ylabel('Vc3')
%% Inciso 12
p_s2 = [-1100.1 (-250 + 0.0948i) (-250 - 0.0948i)]; 
%% Inciso 13
K_pp_s2 = place(A, B, p_s2); 
% -------------------------------------------------------------------------
A_cl_s2 = A - (B * K_pp_s2);
% Definici?n de nueva varaible tipo ss
sys_cl_s2 = ss(A_cl_s2, B, C, D);
% -------------------------------------------------------------------------
[u2,t2] = gensig("square", 1/0.5 , 4, 0.0001);
u2 = u2 + 1;
% -------------------------------------------------------------------------
% Definici?n de vectores
[y1_2,t2,X1_2]=lsim(sys,u2,t2);
[y2_2,t2,X2_2]=lsim(sys_cl_s2,u2,t2);
% Gr?ficas
figure(3); 
clf;
hold on;
subplot(2,1,1);
plot(t2,y1_2);
title("Gr?fica de Sistema Original");
xlabel('t');
ylabel('y Original');
subplot(2,1,2);
plot(t2,y2_2);
title("Gr?fica de Sistema en Lazo Cerrado");
xlabel('t');
ylabel('y Modificado')
suptitle("Salidas de Sistema");
hold off;
% -------------------------------------------------------------------------
% Factor de Escala
Nbar_2 = rscale(sys, K_pp_s2);
[y3_2,t2,X3_2]=lsim(sys_cl_s2, Nbar_2*u2, t2);
% Gr?ficas
figure(4); 
clf;
hold on;
subplot(2,1,1);
plot(t2,y2_2);
title("Gr?fica SIN ESCALAR");
xlabel('t');
ylabel('y');
subplot(2,1,2);
plot(t2,y3_2);
title("Gr?fica ESCALADA");
xlabel('t');
ylabel('y')
suptitle("Comparaci?n de Salidas");
hold off;
% Verificaci?n
linearSystemAnalyzer(Nbar_2 * sys_cl_s2);