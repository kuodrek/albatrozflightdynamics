function [Xp,CF] = Dinamica6GDL(X,U,H,AircraftData)
% FAZER TEXTINHO EXPLICANDO O QUE A FUNÇÃO FAZ E AS REFERENCIAS PELO AMOR
% DE DEUS
geral = AircraftData{2,1};
controle = AircraftData{1,1};
g = geral(1,2);
m = controle(1,5);
Ib = AircraftData{6,1};

u = X(1);
v = X(2);
w = X(3);
p = X(4);
q = X(5);
r = X(6);
phi = X(10);
teta = X(11);
psi = X(12);
dt = U(1);

% Vento no sistema NED (north, east, down)
Nvento = H(1);
Evento = H(2);
Dvento = H(3);
% Matriz de transformação do sistema terrestre móvel para o sistema do corpo (aula 4 de ec nos slides do ITA)
Cbv = [cos(teta)*cos(psi),cos(teta)*sin(psi),-sin(teta);...
       sin(phi)*cos(teta)*cos(psi)-cos(phi)*sin(psi),cos(phi)*cos(psi)+sin(phi)*sin(teta)*sin(psi),sin(phi)*cos(teta);...
       cos(phi)*sin(teta)*cos(psi)+sin(phi)*sin(psi),-sin(phi)*cos(psi)+cos(phi)*sin(teta)*sin(psi),cos(phi)*cos(teta)];
% Vento no sistema do corpo da aeronave (X, Y, Z)
Bvento = Cbv*[Nvento;Evento;Dvento];
uvento = Bvento(1);
vvento = Bvento(2);
wvento = Bvento(3);

u = u + uvento;
v = v + vvento;
w = w + wvento;
X(1) = u;
X(2) = v;
X(3) = w;

V = sqrt(u^2 + v^2 + w^2);
alfa = atan(w/u);
beta = asin(v/V);

[Fa,Ft,Ha,Ht,CF] = fmsolver(X,U,AircraftData);

D = Fa(1);
C = Fa(2);
L = Fa(3);

Xt = Ft(1);
Yt = Ft(2);
Zt = Ft(3);

La = Ha(1);
Ma = Ha(2);
Na = Ha(3);
Lt = Ht(1);
Mt = Ht(2);
Nt = Ht(3);

% Determinação das taxas de velocidade linear (u, v, w) no sistema da aeronave
Cba = [cos(alfa)*cos(beta),-cos(alfa)*sin(beta),-sin(alfa);
       sin(beta),cos(beta),0;
       sin(alfa)*cos(beta),-sin(alfa)*sin(beta),cos(alfa)];
ForcasCorpo = Cba*[-D;C;-L];

Xa = ForcasCorpo(1);
Ya = ForcasCorpo(2);
Za = ForcasCorpo(3);

up = 1/m*(Xa+Xt) - w*q + v*r -g*sin(teta);
vp = 1/m*(Ya+Yt) - u*r + w*p +g*cos(teta)*sin(phi);
wp = 1/m*(Za+Zt) - v*p + u*q +g*cos(teta)*cos(phi);

% taxas angulares
Ix = Ib(1,1);
Iy = Ib(2,2);
Iz = Ib(3,3);
Ixz = Ib(1,3);

gama = Ix*Iz-Ixz^2;
pp = 1/gama*(Ixz*(Ix-Iy+Iz)*p*q-(Iz*(Iz-Iy)+Ixz^2)*q*r+Iz*(La+Lt)+Ixz*(Na+Nt));
qp = 1/Iy*((Iz-Ix)*p*r-Ixz*(p^2-r^2)+(Ma+Mt));
rp = 1/gama*(((Ix-Iy)*Ix+Ixz^2)*p*q-Ixz*(Ix-Iy+Iz)*q*r+Ixz*(La+Lt)+Ix*(Na+Nt));

% Determinação das taxas de posição (x, y e z no referencial fixo)
% A_pos = [cos(teta)*cos(psi), (sin(phi)*sin(teta)*cos(psi)-cos(phi)*sin(psi)), (cos(phi)*sin(teta)*cos(psi)+sin(phi)*sin(psi));...
%          cos(teta)*sin(psi), (sin(phi)*sin(teta)*sin(psi)+cos(phi)*cos(psi)), (cos(phi)*sin(teta)*sin(psi)-sin(phi)*cos(psi));...
%          -sin(teta), sin(phi)*cos(teta) cos(phi)*cos(teta)];
% B_pos = [u; v; w];
% pos = A_pos*B_pos;
% xp_e = pos(1);
% yp_e = pos(2);
% zp_e = -pos(3);

np = u*cos(teta)*cos(psi) + v*(-cos(phi)*sin(psi)+sin(phi)*sin(teta)*cos(psi))+w*(sin(phi)*sin(psi)+cos(phi)*sin(teta)*cos(psi));
ep = u*cos(teta)*sin(psi) + v*(cos(phi)*cos(psi)+sin(phi)*sin(teta)*sin(psi))+w*(-sin(phi)*cos(psi)+cos(phi)*sin(teta)*sin(psi));
hp = u*sin(teta)-v*sin(phi)*cos(teta)-w*cos(phi)*cos(teta);

phip = p + tan(teta)*(q*sin(phi)+r*cos(phi));
tetap = q*cos(phi)-r*sin(phi);
psip = (q*sin(phi)+r*cos(phi))/cos(teta);

Xp = [up vp wp pp qp rp np ep hp phip tetap psip];
end