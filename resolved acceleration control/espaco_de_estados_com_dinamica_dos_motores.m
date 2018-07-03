function [sys,x0,str,ts] = espaco_de_estados(t,x,u,flag,r,m,Jw,J,L)
% Sistema do robô omnidirecional Axebot em espaço de estados.  

% Definir parâmetros
a1 =  2*r^2*m/(2*r^2*m+3*Jw); % não está sendo utilizado (foi parte do cálculo de outras constantes)
a2 = 3*Jw/(2*r^2*m+3*Jw);
b1 = r/(2*r^2*m+3*Jw);
b2 = r*L/(J*r^2+3*Jw*L^2);

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,a2,b1,b2);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
% end csfunc

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes()

sizes = simsizes;
sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [0 0 pi/2 0 0 0];
str = [];
ts  = [-1 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,a2,b1,b2)
xq = x(1);
yq = x(2);
phi = x(3);
dxdt = x(4);
dydt = x(5);
dphidt = x(6);

beta1 = sin(phi)-sqrt (3)*cos(phi);
beta2 = sin(phi)+sqrt(3)*cos(phi);
beta3 = sqrt(3)*sin(phi)+cos(phi);
beta4 = cos(phi)-sqrt(3)*sin(phi);

% u(1)-> ddxodtt, u(2)-> ddyodtt, u(3)-> ddphidtt
% X = u(1)+a2*dphidt*dydt;
% Y = u(2)-a2*dphidt*dxdt; 
% PHI = u(3);

% u1 = sin(phi)/(6*b1)*X+cos(phi)/(6*b1)*Y+1/(3*b2)*PHI;
% u2 = -beta1/(12*b1)*X+beta3/(12*b1)*Y+1/(3*b2)*PHI;
% u3 = -beta2/(12*b1)*X+beta4/(12*b1)*Y+1/(3*b2)*PHI;
% Entradar vai ser o pulso
u1 = u(1); u2 = u(2); u3 = u(3);

ddxdtt = -a2*dphidt*dydt+4*sin(phi)*b1*u1-2*b1*beta1*u2-2*b1*beta2*u3;
ddydtt = a2*dphidt*dxdt-4*cos(phi)*b1*u1+2*beta3*b1*u2+2*beta4*b1*u3;
ddphidtt = -b2*(u1+u2+u3);

 sys = [dxdt dydt dphidt ddxdtt ddydtt ddphidtt];

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
dxdt = x(4);
dydt = x(5);
phi = x(3);

sys = [dxdt dydt phi];

% end mdlOutputs
