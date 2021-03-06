function [Fa,Ft,Ha,Ht,CF] = fmsolver(X,U,AircraftData)
u = X(1);
v = X(2);
w = X(3);
p = X(4);
q = X(5);
r = X(6);

V = sqrt(u^2 + v^2 + w^2);
alfa = atan(w/u);
beta = asin(v/V);

% Dados gerais
geral = AircraftData{2,1};
rho = geral(1,1);
g = geral(1,2);
Sref = geral(1,3);
bref = geral(1,4);
cref = geral(1,5);
nh = geral(1,6);
yp = geral(1,7);
zp = geral(1,8);
lp = geral(1,9);
xcg = geral(1,10);
zcg = geral(1,11);
zh = geral(1,12);

% Dados das Superf?cies
superficies = AircraftData{3,1};
Sw = superficies(1,1);
CLaw = superficies(1,2);
CL0w = superficies(1,3);
Cmacw = superficies(1,4);
% damin = superficies(1,5);
% damax = superficies(1,6);
iw = superficies(1,7);
depsilondalpha = superficies(1,8);
epsilon0 = superficies(1,9);
CDw_c = [superficies(1,10) superficies(1,11) superficies(1,12)];
xac = superficies(1,13);
% Dados EH
Sh = superficies(2,1);
CLah = superficies(2,2);
CL0h = superficies(2,3);
Cmach = superficies(2,4);
% demin = superficies(2,5);
% demax = superficies(2,6);
ih = superficies(2,7);
Vh = superficies(2,8);
CDh_c = [superficies(2,10) superficies(2,11) superficies(2,12)];
% % Dados EV
% drmin = superficies(3,5);
% drmax = superficies(3,6);
% % Dados miscel?neos
% dtmin = superficies(4,5);
% dtmax = superficies(4,6);

% Convers?o da deflex?o dos controles
dt = U(1);
de = U(2);
da = U(3);
dr = U(4);

motorInput = AircraftData{5,1};
T = max(tracao(V,dt,motorInput),0);
% T = dt;
T = T*g;
% Derivadas da aeronave
derivadas = AircraftData{4,1};
% beta
CYb = derivadas(1,1);
Clb = derivadas(1,2);
Cnb = derivadas(1,3);
% p
CYp = derivadas(2,1);
Clp = derivadas(2,2);
Cnp = derivadas(2,3);
% q
CLq = derivadas(3,1);
Cmq = derivadas(3,2);
% r
CYr = derivadas(4,1);
Clr = derivadas(4,2);
Cnr = derivadas(4,3);
% Aileron
CYda = derivadas(5,1);
Clda = derivadas(5,2);
Cnda = derivadas(5,3);
% Profundor
CLde = derivadas(6,1);
Cmde = derivadas(6,2);
% Leme
CYdr = derivadas(7,1);
Cldr = derivadas(7,2);
Cndr = derivadas(7,3);

% C?lculo da press?o din?mica
Q = 0.5*rho*V^2;

Ft = [T 0 0];
Lt = T*yp*sin(lp);
Mt = T*zp*cos(lp);
Nt = T*yp*cos(lp);
Ht = [Lt Mt Nt];

alfah = alfa*(1-depsilondalpha) - epsilon0 + ih;
CDw = CDw_c(1)*alfa^2 + CDw_c(2)*alfa + CDw_c(3);
CDh = CDh_c(1)*alfah^2 + CDh_c(2)*alfah + CDh_c(3);
CD = CDw + nh*Sh/Sw*CDh;
% EQUA??O DE FOR?A EM X
D = Q*Sref*CD;
CY = CYp*p*bref/(2*V) + CYr*r*bref/(2*V) + CYb*beta + CYda*da + CYdr*dr;
C = Q*Sref*CY;
CL = CLaw*(alfa+iw) + CL0w + nh*Sh/Sw*(CLah*alfah + CL0h) + CLde*de + CLq*q*cref/(2*V);
L = Q*Sref*CL;

Fa = [D C L];

Cl = Clp*p*bref/(2*V) + Clr*r*bref/(2*V) + Clb*beta + Clda*da + Cldr*dr;
La = Q*Sref*bref*Cl;

Cmcgw = (xcg-xac)*((CL0w + CLaw*(alfa+iw))*cos(alfa+iw)+CDw*sin(alfa+iw)) + zcg/cref*((CL0w + CLaw*(alfa+iw))*sin(alfa+iw)-CDw*cos(alfa+iw)) + Cmacw;
Vhz = zh*Sh/(cref*Sref);
Cmcgh = -nh*Vh*((CL0h+CLah*alfah)*cos(alfah)+CDh*sin(alfah)) -nh*Vhz*(CDh*cos(alfah)-(CL0h+CLah*alfah)*sin(alfah))+ nh*Sh/Sw*Cmach;
% Cm = Cmacw + (CL0w + CLaw*(alfa+iw))*(xcg-xac) + nh*Sh/Sw*Cmach - nh*Vh*(CL0h+CLah*alfah) + Cmde*de + Cmq*q/(2*V);
Cm =  Cmcgw + Cmcgh + Cmde*de + Cmq*q/(2*V);
Ma = Q*Sref*cref*Cm;

Cn = Cnp*p*bref/(2*V) + Cnr*r*bref/(2*V) + Cnb*beta + Cnda*da + Cndr*dr;
Na = Q*Sref*bref*Cn;

Ha = [La Ma Na];
CF = [CD CY CL Cl Cm Cn];
end