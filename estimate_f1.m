%% It is used to estimate nonlinear values in the prediction domain
function Ft = estimate_f1(x_k,A,k,Np)
%****************************************************************
  %Function: Estimate the prediction sequence of nonlinear terms in the prediction domain
  %Calling format：Ft=estimate_f(x_k,u_k,Np)
  %Input parameters: the current state of x_k, the length of the prediction domain of the Np system
  %         A,parameters, k(4 in total)
  %Output parameter: prediction sequence of nonlinear terms
%****************************************************************
Ft = [];
xk = zeros(2,Np);
xk(:,1) = x_k;
for i=1:Np
    f(:,i)=[0;-0.316*exp(-0.1*xk(1,i))*xk(1,i)/(2*xk(1,i)*xk(1,i)-xk(2,i)+0.1)-0.48*cos((k+i-1)*pi/5)*xk(2,i)];
    xk(:,i+1)=A*xk(:,i)+f(:,i);
    Ft = [Ft;f(:,i)];
end