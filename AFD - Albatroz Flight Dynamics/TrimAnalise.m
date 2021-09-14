function CostFunction = TrimAnalise(TrimInputs)
global data_check
global AircraftData
global SetupVoo
if isempty(data_check) == 1
    data_check = 1;
    [data,setup] = datasetup();
    AircraftData = data;
    SetupVoo = setup;
end
tipo = SetupVoo(1);

if tipo == 1 % Voo de cruzeiro longitudinal
    % Variáveis de estado
    V = SetupVoo(2);
    alfa = TrimInputs(1);
    beta = 0;
    
    % Determinação das velocidades lineares no referencial da aeronave
    u = V*cos(beta)*cos(alfa);
    v = V*sin(beta);
    w = V*cos(beta)*sin(alfa);
    p = 0;
    q = 0;
    r = 0;
    x_e = 0;
    y_e = 0;
    h = SetupVoo(6);
    phi = 0;
    teta = alfa;
    psi = 0;
    X = [u v w p q r x_e y_e h phi teta psi]';
    
    % Variáveis de controle
    dt = TrimInputs(2);
    de = TrimInputs(3);
    da = 0;
    dr = 0;
    U = [dt de da dr]';
    H = [0 0 0];
    Xp = Dinamica6GDL(X,U,H,AircraftData);
elseif tipo == 2 % Voo de cruzeiro nivelado
    H = [0 0 0];
    Xp = Dinamica6GDL(X,U,H,AircraftData);
elseif tipo == 3 % Voo retilíneo (assimétrico)
    H = [0 0 0];
    Xp = Dinamica6GDL(X,U,H,AircraftData);
elseif tipo == 4 % Push-up / Pull-over
    geral = AircraftData{2,1};
    g = geral(2);
    m = SetupVoo(5);
    n = SetupVoo(8);
    motorInput = AircraftData{5,1};
    % Variáveis de estado
    V = SetupVoo(2);
    alfa = TrimInputs(1);
    beta = 0;
    
    % Determinação das velocidades lineares no referencial da aeronave
    u = V*cos(beta)*cos(alfa);
    v = V*sin(beta);
    w = V*cos(beta)*sin(alfa);
    p = 0;
    r = 0;
    x = 0;
    y = 0;
    h = SetupVoo(6);
    phi = 0;
    teta = alfa;
    psi = 0;
    
    % Variáveis de controle
    dt = TrimInputs(2);
    de = TrimInputs(3);
    da = 0;
    dr = 0;
    U = [dt de da dr];
    H = [0 0 0];
    % Determinação da velocidade de arfagem
    T = tracao(V,dt,motorInput);
    Xt = T*g;
    Zt = 0;
    q = 1/(V*m*cos(beta))*(m*g*(n-1)+Zt*cos(alfa)-Xt*sin(alfa));
    X = [u v w p q r x y h phi teta psi];
    Xp = Dinamica6GDL(X,U,H,AircraftData);
elseif tipo == 5 % Curva em regime permanente
    H = [0 0 0];
    Xp = Dinamica6GDL(X,U,H,AircraftData);
elseif tipo == 6 % Curva coordenada
    H = [0 0 0];
    Xp = Dinamica6GDL(X,U,H,AircraftData);
elseif tipo == 7 % Controle direcional em solo
    H = [0 0 0];
    Xp = Dinamica6GDL(X,U,H,AircraftData);
elseif tipo == 8 % Decolagem (plano longitudinal)
    H = [0 0 0];
    Xp = Dinamica6GDL(X,U,H,AircraftData);
end

% Xp = [up vp wp pp qp rp xp_e yp_e zp_e phip tetap psip]';
up = Xp(1);
vp = Xp(2);
wp = Xp(3);
pp = Xp(4);
qp = Xp(5);
rp = Xp(6);

xd_vetor = [up vp wp pp qp rp]';
pesos = [1 1 1 1 1 1];
W = zeros(6,6);
for i=1:6
    W(i,i) = pesos(i);
end
CostFunction = xd_vetor'*W*xd_vetor;
end