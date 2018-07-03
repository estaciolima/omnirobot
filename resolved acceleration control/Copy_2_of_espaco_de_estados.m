function [sys,x0,str,ts] = espaco_de_estados(t,x,u,flag,r,m,Jw,J,L) 
% Sistema do robô omnidirecional Axebot em espaço de estados.  

% Definir parâmetros
a1 =  2*r^2*m/(2*r^2*m+3*Jw); % não está sendo utilizado (foi parte do cálculo de outras constantes)
a2 = 3*Jw/(2*r^2*m+3*Jw);
b1 = r/(2*r^2*m+3*Jw);
b2 = r*L/(J*r^2+3*Jw*L^2);
alpha1 = 0;
alpha2 = 120*pi/180;
alpha3 = 240*pi/180;
l1 = L;
l2 = L;
l3 = L;

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
    sys=mdlOutputs(t,x,u,alpha1,alpha2,alpha3,l1,l2,l3,r);

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
sizes.NumOutputs     = 6;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [0 0 pi/2 0 0 0];
str = [];
ts  = [0 0];
end
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

beta1 = sin(phi)-sqrt(3)*cos(phi);
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
u1 = u(1); u2 = u(2); u3 = u(3);

ddxdtt = -a2*dphidt*dydt+4*sin(phi)*b1*u1-2*b1*beta1*u2-2*b1*beta2*u3;
ddydtt = a2*dphidt*dxdt-4*cos(phi)*b1*u1+2*beta3*b1*u2+2*beta4*b1*u3;
ddphidtt = -b2*(u1+u2+u3);

sys = [dxdt dydt dphidt ddxdtt ddydtt ddphidtt];
end
% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,alpha1,alpha2,alpha3,l1,l2,l3,r)
phi = x(3);
dxdt = x(4);
dydt = x(5);
dphidt = x(6);

Jf1 = [sin(alpha1) -cos(alpha1) -l1
       sin(alpha2) -cos(alpha2) -l2
       sin(alpha3) -cos(alpha3) -l3];

R = [cos(phi) sin(phi) 0
    -sin(phi) cos(phi) 0
    0 0 1];

% Velocidades lineares das rodas
v = Jf1*R*[dxdt dydt dphidt]';

w1 = v(1)/r;
w2 = v(2)/r;
w3 = v(3)/r;

sys = [dxdt dydt phi w1 w2 w3];
end
% end mdlOutputs