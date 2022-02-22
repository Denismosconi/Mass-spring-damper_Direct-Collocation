function J = DC_ObjFun(x,auxdata)

%Computa o valor da fun��o objetivo
% x � o vetor contendo os estados e o sinal de controle atuais
% f � a integral dos quadrados dos sinais de controle

%Extraindo o step time e o n�mero de n�s do auxdata
dt = auxdata.dt;
n = auxdata.n;

%Extraindo o vetor de controle do vetor x
u = x(2*n+1:3*n);

%Calculando a integral do quadrado dos elementos que comp�em o sinal de
%controle u
Integral = trapz(u.^2)*dt;


%Determinando a fun��o objetivo, onde R � um ganho, que for�a o sinal de
%controle a ser mais ou menos reduzido
R = 1e2;
J = R*Integral;

end