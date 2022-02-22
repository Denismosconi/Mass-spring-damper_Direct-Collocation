function [c,ceq] = DC_ConFun(x,auxdata)

%Esta função determina as constrains (restrições) para o otimizador

%Extraindo o step time e o número de nós do auxdata
dt = auxdata.dt;
n = auxdata.n;

% Não há inegualidades não lineares, então tal vetor é retornado vazio 
c = [];

%Extraindo os estados e controle do vetor x
s = x(1:n);
v = x(n+1:2*n);
u = x(2*n+1:3*n);

%Computando a derivada dos estados s e v pelo método Backward de Euler
ds = diff(s)/dt;
dv = diff(v)/dt;
dx = [ds; dv];

%Computando a derivada dos estados s e v por dx = f(x,u)
dxf = computeXdot(s, v, u, auxdata);

%Criando o vetor de restrições igualitárias
ceq = dx-dxf;

%Até aqui, a otimização acontece reduzindo a função objetivo e atendendo à
%restrição de dx = f(x,u). Porém nós ainda queremos que a massa siga uma
%trajetória que passe por alguns pontos determinados. Nós desejamos que o
%ponto inicial seja o repouso, que no meio do movimento a massa passe pela
%posição 0.2 m e ao final do movimento retorne para o ponto de repouso.
%Para garantir esse movimento, nós colocamos esses pontos como restrições,
%pois isso força o otimizador a produzir uma resposta que cotenha esses
%pontos.

%Forçando a posição e a velocidade iniciais a 0
ceq(end+1,1) = s(1,1) - (0.0);
ceq(end+1,1) = v(1,1) - (0.0);

%Forçando a posição intermediária a 0.25 m e a velocidade intermediária a 0
ceq(end+1,1) = s(round(end/2),1) - (0.25);
ceq(end+1,1) = v(round(end/2),1) - (0.0);

%Forçando a posição e a velocidade finais a 0
ceq(end+1,1) = s(end,1) - (0.0);
ceq(end+1,1) = v(end,1) - (0.0);

%Forçando os controles finais a serem iguais aos iniciais
%ceq(end+1,1) = u(end,1) - u(1,1);

%Forçanco o controle final a ser zero
ceq(end+1,1) = u(end,1) - (0.0);

end
















