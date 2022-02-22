function J = DC_ObjFun(x,auxdata)

%Computa o valor da função objetivo
% x é o vetor contendo os estados e o sinal de controle atuais
% f é a integral dos quadrados dos sinais de controle

%Extraindo o step time e o número de nós do auxdata
dt = auxdata.dt;
n = auxdata.n;

%Extraindo o vetor de controle do vetor x
u = x(2*n+1:3*n);

%Calculando a integral do quadrado dos elementos que compõem o sinal de
%controle u
Integral = trapz(u.^2)*dt;


%Determinando a função objetivo, onde R é um ganho, que força o sinal de
%controle a ser mais ou menos reduzido
R = 1e2;
J = R*Integral;

end