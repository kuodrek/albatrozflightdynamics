# ALBATROZ FLIGHT DYNAMICS V1

O AFD é um conjunto de funções escritas em MatLab que possibilitam a simulação do voo de uma aeronave em 6 graus de liberdade (6DOF).\
O Algoritmo é baseado nas equações do livro Aircraft Control And Simulation (Stevens Et al.) e utiliza integração numérica pelo método de Runge-Kutta de 4a ordem.\
O programa utiliza aproximação linear dos coeficientes aerodinâmicos porém a função pode ser trocada para modelos mais sofisticados caso necessário.\
As Equações de Dinâmica de Voo são totalmente acopladas e não-lineares.

# INSTRUÇÕES
A planilha dinamicadados.xlsx recebe os dados da aeronave (Valores geométricos do avião, Coeficientes aerodinâmicos e Condições de contorno).\
O programa pode ser executado através da função dinamicamain.m. Dentro do Loop da simulação é possível simular as perturbações de Controle e rajadas de vento.\
O AFD também conta com funções para se encontrar o estado de equilíbrio da Aeronave dada uma determinada condição através do método fminsearch.\
\
É possível ajustar os parâmetros de simulação como timestep (variável h), tempo inicial (t_inicial) e tempo_final (t_final).\
Ao final da simulação, gráficos são plotados ilustrando as variáveis de estado da aeronave para uma dada perturbação inserida pelo usuário.\

# SIMULAÇÃO DE PERTURBAÇÕES
Vetor U: Vetor de deflexão das superfícies móveis do avião\
U(1) = RPM do motor\
U(2) = Deflexão do profundor (rad)\
U(3) = Deflexão do aileron (rad)\
U(4) = Deflexão do leme (rad)\
\
Vetor H: Vetor de velocidades do vento no eixo da aeronave\
H(1) = Componente da velocidade do vento no eixo X da aeronave\
H(2) = Componente da velocidade do vento no eixo Y da aeronave\
H(3) = Componente da velocidade do vento no eixo Z da aeronave\
\
O vetor Solução contém os estados que descrevem a aeronave (velocidades lineares e angulares, deslocamento e ângulos de trajetória) calculados ponto a ponto

# INFORMAÇÕES ADICIONAIS
Os arquivos em .txt são algumas derivadas aerodinâmicas retiradas do AVL que foram utilizadas para verificação do comportamento do programa. Sinta-se a vontade para mexer nos valores!
