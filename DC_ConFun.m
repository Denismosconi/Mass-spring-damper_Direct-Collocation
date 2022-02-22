function [c,ceq] = DC_ConFun(x,auxdata)

%Esta fun��o determina as constrains (restri��es) para o otimizador

%Extraindo o step time e o n�mero de n�s do auxdata
dt = auxdata.dt;
n = auxdata.n;

% N�o h� inegualidades n�o lineares, ent�o tal vetor � retornado vazio 
c = [];

%Extraindo os estados e controle do vetor x
s = x(1:n);
v = x(n+1:2*n);
u = x(2*n+1:3*n);

%Computando a derivada dos estados s e v pelo m�todo Backward de Euler
ds = diff(s)/dt;
dv = diff(v)/dt;
dx = [ds; dv];

%Computando a derivada dos estados s e v por dx = f(x,u)
dxf = computeXdot(s, v, u, auxdata);

%Criando o vetor de restri��es igualit�rias
ceq = dx-dxf;

%At� aqui, a otimiza��o acontece reduzindo a fun��o objetivo e atendendo �
%restri��o de dx = f(x,u). Por�m n�s ainda queremos que a massa siga uma
%trajet�ria que passe por alguns pontos determinados. N�s desejamos que o
%ponto inicial seja o repouso, que no meio do movimento a massa passe pela
%posi��o 0.2 m e ao final do movimento retorne para o ponto de repouso.
%Para garantir esse movimento, n�s colocamos esses pontos como restri��es,
%pois isso for�a o otimizador a produzir uma resposta que cotenha esses
%pontos.

%For�ando a posi��o e a velocidade iniciais a 0
ceq(end+1,1) = s(1,1) - (0.0);
ceq(end+1,1) = v(1,1) - (0.0);

%For�ando a posi��o intermedi�ria a 0.25 m e a velocidade intermedi�ria a 0
ceq(end+1,1) = s(round(end/2),1) - (0.25);
ceq(end+1,1) = v(round(end/2),1) - (0.0);

%For�ando a posi��o e a velocidade finais a 0
ceq(end+1,1) = s(end,1) - (0.0);
ceq(end+1,1) = v(end,1) - (0.0);

%For�ando os controles finais a serem iguais aos iniciais
%ceq(end+1,1) = u(end,1) - u(1,1);

%For�anco o controle final a ser zero
ceq(end+1,1) = u(end,1) - (0.0);

end
















