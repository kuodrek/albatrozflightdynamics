function [X_0,U_0] = maintrim()
%% Chamar datasetup para obter os dados da aeronave e condições de voo
[data,setup] = datasetup();
AircraftData = data;
superficies = AircraftData{3,1};
dtmin = superficies(4,5);
dtmax = superficies(4,6);
demin = superficies(2,5);
demax = superficies(2,6);
damin = superficies(1,5);
damax = superficies(1,6);
drmin = superficies(3,5);
drmax = superficies(3,6);
SetupVoo = setup;
tipo = SetupVoo(1);
V = SetupVoo(2);
h0 = SetupVoo(6);
trim = @TrimAnalise;

%% Casos de trimagem
if tipo == 1  % Voo de cruzeiro longitudinal
    x0 = [0*pi/180, 5000, 0];
    lb = [-20*pi/180, dtmin, demin];
    ub = [20*pi/180, dtmax, demax];
    options = optimoptions('patternsearch');
    options.MaxIterations = 10000;
%     options.Display = 'off';
    trim_setup = patternsearch(trim,x0,[],[],[],[],lb,ub,[],options);
    
    alfa = trim_setup(1);
    beta = 0;
    % Determinação das velocidades lineares no referencial da aeronave
    u = V*cos(beta)*cos(alfa);
    v = V*sin(beta);
    w = V*cos(beta)*sin(alfa);
    teta = alfa;
    dt = trim_setup(2);
    de = trim_setup(3);
    
    X_0 = [u v w 0 0 0 0 0 h0 0 teta 0];
    U_0 = [dt de 0 0];
elseif tipo == 2
elseif tipo == 3
elseif tipo == 4 % Push-up / Pull-over
    geral = AircraftData{2,1};
    g = geral(2);
    m = SetupVoo(5);
    n = SetupVoo(8);
    motorInput = AircraftData{5,1};
    x0 = [0 dtmin demin];
    lb = [-20*pi/180, dtmin, demin];
    ub = [20*pi/180, dtmax, demax];
    options = optimoptions('patternsearch');
    options.MaxIterations = 1000;
    options.Display = 'off';
    trim_setup = patternsearch(trim,x0,[],[],[],[],lb,ub,[],options);
    
    % Determinação da velocidade de arfagem
    alfa = trim_setup(1);
    beta = 0;
    dt = trim_setup(2);
    T = tracao(V,dt,motorInput);
    Xt = T*g;
    Zt = 0;
    q = 1/(V*m*cos(beta))*(m*g*(n-1)+Zt*cos(alfa)-Xt*sin(alfa));
    
    X_0 = [SetupVoo(2) 0 trim_setup(1) 0 q 0 0 0 h0 0 trim_setup(1) 0];
    U_0 = [trim_setup(2) trim_setup(3) 0 0];
elseif tipo == 5
elseif tipo == 6
elseif tipo == 7
elseif tipo == 8
end