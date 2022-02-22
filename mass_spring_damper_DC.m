%Este programa utiliza a direct collocation approach para simular o
%controle ótimo aplicado a um sistema massa-mola-amortecedor.

%O sistema massa-mola-amortecedor possui 2 estados: posição e velocidade, e
%apenas 1 controle: força aplicada

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Limpando as variáveis
close all;
clear;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Determinando o número de nós (collocation points)
n = 100;

%Tempo final em segundos
tf = 1.5;

%Step time
dt = tf/(n-1);

%Vetor de tempo
time = dt*(0:n-1)';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Determinando os números de estados e controles para o sistema
%massa-mola-amortecedor estudado.
Nstates = 2; %2 estados: posição e velocidade
Ncontrols = 1; %1 controle: força aplicada

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Criando uma estrutura de dados auxiliar a ser passada para o otimizador
auxdata.time = time;
auxdata.n = n;
auxdata.dt = dt;
auxdata.Nstates = Nstates;
auxdata.Ncontrols = Ncontrols;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Inicializando as variáveis posição, velocidade e controle. Após a
%inicialização, todos os estados e controle são agrupados em um único vetor
%o qual representa as variáveis a serem otimizadas.

%Inicializando posição (s) com valores entre 0 e 0.1 m
s = rand(n,1)*0.1;

%Inicializando velocidade (v) com valores entre 0 e 0.1 m/s
v = rand(n,1)*0.1;

%Inicializando o controle (u) com valores entre 0 e 1 N
u = rand(n,1);

%Criando o vetor X0 que conterá as variáves inicializadas acima
X0 = [s; v; u];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Determinando os limites inferior e superior de cada uma das variáveis
%(posição, velocidade e controle)

%Posição
Pos_LB(1:n) = -0.5;
Pos_UB(1:n) = 0.5;

%Velocidade
Vel_LB(1:n) = -2.0;
Vel_UB(1:n) = 2.0;

%Controle
U_LB(1:n) = -5.0;
U_UB(1:n) = 5.0;

lb = [Pos_LB Vel_LB U_LB]';
ub = [Pos_UB Vel_UB U_UB]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definindo os handlers para a função objetivo e a função de restrições
objfun = @(x)DC_ObjFun(x,auxdata);
confun = @(x)DC_ConFun(x,auxdata);

%Definindo algumas opções para o otimizador fmincon
options = optimset('algorithm','interior-point','TolFun',1e-4,'TolX',1e-4, ...
                   'TolCon',1e-4,'FinDiffType','forward','MaxFunEvals',1e5, ...
               'Hessian','bfgs','display','iter');

%Disparando um timer para medir quanto tempo levou para acontecer a
%simulação
tic;

%Iniciando a otimização
[Xopt,fval,exitflag,output] = fmincon(objfun,X0,[],[],[],[],lb,ub,confun,options);

%Parando o timer e medindo o tempo de simulação em minutos
runtime = (toc)/60;

%Extraindo os estados e controle ótimos do vetor Xopt
sopt = Xopt(1:n);
vopt = Xopt(n+1:2*n);
uopt = Xopt(2*n+1:3*n);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plotando os resultados

%Posição
figure(1)
plot(time, sopt, 'b', 'Linewidth', 2.0)
hold on
plot(time, s, 'r', 'Linewidth', 2.0)
grid;
title('Position', 'FontSize', 16); 
xlabel('Time [s]', 'FontSize', 16); 
ylabel('Position [m]', 'FontSize', 16);
legend('Optimal', 'Guess' ,'Location','southeast', 'Orientation','vertical');
set(gca, 'FontSize', 16);
ylim([-0.5 0.5]);
yticks(-0.5:0.25:0.5);

%Velocidade
figure(2)
plot(time, vopt, 'k', 'Linewidth', 2.0)
hold on
plot(time, v, 'g', 'Linewidth', 2.0)
grid;
title('Velocity', 'FontSize', 16); 
xlabel('Time [s]', 'FontSize', 16); 
ylabel('Velocity [m/s]', 'FontSize', 16);
legend('Optimal', 'Guess' ,'Location','southeast', 'Orientation','vertical');
set(gca, 'FontSize', 16);
ylim([-1 1]);

%Controle
figure(3)
plot(time, uopt, 'k--', 'Linewidth', 2.0)
hold on
plot(time, u, 'm', 'Linewidth', 2.0)
grid;
title('Control', 'FontSize', 16); 
xlabel('Time [s]', 'FontSize', 16); 
ylabel('Control [N]', 'FontSize', 16);
legend('Optimal', 'Guess' ,'Location','southeast', 'Orientation','vertical');
set(gca, 'FontSize', 16);
ylim([-5 5]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Imprimindo informações finais
disp( ['Optimization: elapsed time  = ' num2str(runtime) ' s'])
disp( ['Optimization: min objective fun value = ' num2str(fval)])
disp( ['Optimization: number iterations  = ' num2str(output.iterations)])
disp( ['Optimization: number obj func evals  = ' num2str(output.funcCount)])























