function dxf = computeXdot(s, v, u, auxdata)

%Esta fun��o calcula a derivada dos estados de acordo com a fun��o dxf=f(x,u)
% Neste caso nos utilizaos dxf para indicar que o dx veio da fun��o acima e
% n�o do m�todo de Euler como na fun��o DC_ConFun

%Par�metros do sistema massa-mola-amortecedor
m = 1;
K = 1;
B = 0.2;

%Extraindo o n�mero de n�s do auxdata
n = auxdata.n;

%Determinando a derivada do estado 1, ou seja, a derivada da posi��o, que �
%a velocidade
dsf = v(1:n-1);

%Determinando a derivada do estado 2, ou seja, a derivada da velocidade, que �
%a acelera��o
dvf = u(1:n-1)-s(1:n-1)*(K/m)-v(1:n-1)*(B/m);

dxf = [dsf;dvf];

end