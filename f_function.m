%% Nonlinear term computation
function f = f_function(x,k)
%****************************************************************
  %Function: Calculate the output value of the nonlinear term
  %Calling format：Ft=f_function(x,A,k)
  %Input parameters: x current latest status,k current simulation step
  %Output parameter: Output value of nonlinear item
%****************************************************************
f = [0;-0.216*exp(-x(1,1))*x(1,1)-0.048*cos(k*pi/5)*x(2,1)]; 