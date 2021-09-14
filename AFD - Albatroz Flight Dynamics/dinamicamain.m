% DINAMICA 6 GDL SETUP
clear
clc
clf
%% Trim da aeronave
disp('Chamando a função de minimização de custo para encontrar o trim da aeronave...');
[X_0,U_0] = maintrim();
H_0 = [0 0 0];
disp('Fim da minimização.')
%% Chamar datasetup para os dados da aeronave
disp('Chamando datasetup')
global data_check
global AircraftData
global SetupVoo
if isempty(data_check) == 1
    data_check = 1;
    [data,setup] = datasetup();
    AircraftData = data;
    SetupVoo = setup;
end
disp('Dados adquiridos!')
%% Solver numérico
disp('INICIANDO SIMULAÇÃO')
t_inicial = 0;
t_final = 20;
h=20e-3;
n_pto = (t_final-t_inicial)/h+1;
vet_t=linspace(t_inicial,t_final,n_pto);   % Vetor dos tempos amostrados
solucao = zeros(n_pto,12);
Xp_vetor = zeros(n_pto,12);
X = X_0;
U = U_0;
H = H_0;
check1 = 0;
check2 = 0;
for i=1:n_pto
    solucao(i,:) = X;
    t = vet_t(i);
    if t > 0 && check1 == 0
%         dt_comando = 0.1;
%         U(1) = U_0(1) + dt_comando;
%         de_comando = 2*pi/180;
%         U(2) = U_0(2) + de_comando;
%         da_comando = 1*pi/180;
%         U(3) = U_0(3) + da_comando;
%         dr_comando = 15*pi/180;
%         U(4) = U_0(4) + dr_comando;
        check1 = 1;
    end
%     if t > 2 && check2 == 0
%         dt_comando = -0.1;
%         U(1) = U_0(1) + dt_comando;
%         de_comando = -2*pi/180;
%         U(2) = U_0(2) + de_comando;
%         da_comando = 5*pi/180;
%         U(3) = U_0(3) + da_comando;
%         dr_comando = 5*pi/180;
%         U(4) = U_0(4) + dr_comando;
%         check2 = 1;
%     end
    if t > 2
        U = U_0;
    end
    U_vet(i,:) = U;
    K1=Dinamica6GDL(X,U,H,AircraftData);
    [~,CF]=Dinamica6GDL(X,U,H,AircraftData);
    CF_vetor(i,:) = CF;
    K2=Dinamica6GDL(X+h/2*K1,U,H,AircraftData);
    K3=Dinamica6GDL(X+h/2*K2,U,H,AircraftData);
    K4=Dinamica6GDL(X+h*K3,U,H,AircraftData);
    X=X+h/6*(K1+2*K2+2*K3+K4);
    Xp_vetor(i,:) = K1;
end
%% Pós processamento
t_excel = vet_t';
disp('Fim da simulação. Plotando gráficos...')
%  Xp = [up vp wp pp qp rp xp_e yp_e zp_e phip tetap psip]';
u = solucao(:,1);
v = solucao(:,2);
w = solucao(:,3);
V = sqrt(u.^2+v.^2+w.^2);
alfa = atan(w./u)*180/pi;
beta = atan(v./V)*180/pi;
p = solucao(:,4)*180/pi;
q = solucao(:,5)*180/pi;
r = solucao(:,6)*180/pi;
x = solucao(:,7);
y = solucao(:,8);
z = solucao(:,9);
phi = solucao(:,10)*180/pi;
teta = solucao(:,11)*180/pi;
psi = solucao(:,12)*180/pi;

figure(1)
subplot(2,3,1);
V_0 = V(1)*ones(size(V,1),size(V,2));
plot(vet_t,V);
title('V x t')
xlim([0 t_final])
ylim([0 1.2*V(1)])

subplot(2,3,2);
alfa_0 = alfa(1)*ones(size(alfa,1),size(alfa,2));
hold on
plot(vet_t,alfa);
title('Alfa (aeronave) x t')
xlim([0 t_final])
ylim([-20 20])
plot(vet_t,U_vet(:,2)*180/pi,'r');
hold off

subplot(2,3,3);
q_0 = q(1)*ones(size(q,1),size(q,2));
plot(vet_t,q);
title('q x t')
xlim([0 t_final])
ylim([-20 20])

subplot(2,3,4);
plot(vet_t,z);
title('z x t')
xlim([0 t_final])
ylim([0 100])

subplot(2,3,5);
teta_0 = teta(1)*ones(size(teta,1),size(teta,2));
plot(vet_t,teta);
title('teta x t')
xlim([0 t_final])
ylim([-30 30])

figure(2)
subplot(2,3,1);
plot(vet_t,beta);
title('beta x t')
xlim([0 t_final])
ylim([-10 10])

subplot(2,3,2);
plot(vet_t,p);
title('p x t')
xlim([0 t_final])
ylim([-50 50])

subplot(2,3,3);
plot(vet_t,r);
title('r x t')
xlim([0 t_final])
ylim([-50 50])

subplot(2,3,4);
plot(vet_t,phi);
title('phi x t')
xlim([0 t_final])
ylim([-180 180])

subplot(2,3,5);
plot(vet_t,v);
title('v x t')
xlim([0 t_final])
ylim([-10 10])


subplot(2,3,6);
plot(vet_t,psi);
title('psi x t')
xlim([0 t_final])
ylim([-180 180])