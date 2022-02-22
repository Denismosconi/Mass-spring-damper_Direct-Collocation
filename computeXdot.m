function dxf = computeXdot(s, v, u, auxdata)

%Esta função calcula a derivada dos estados de acordo com a função dxf=f(x,u)
% Neste caso nos utilizaos dxf para indicar que o dx veio da função acima e
% não do método de Euler como na função DC_ConFun

%Parâmetros do sistema massa-mola-amortecedor
m = 1;
K = 1;
B = 0.2;

%Extraindo o número de nós do auxdata
n = auxdata.n;

%Determinando a derivada do estado 1, ou seja, a derivada da posição, que é
%a velocidade
dsf = v(1:n-1);

%Determinando a derivada do estado 2, ou seja, a derivada da velocidade, que é
%a aceleração
dvf = u(1:n-1)-s(1:n-1)*(K/m)-v(1:n-1)*(B/m);

dxf = [dsf;dvf];

end