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
