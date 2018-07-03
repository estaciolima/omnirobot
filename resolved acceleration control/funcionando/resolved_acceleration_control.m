function [sys,x0,str,ts] = resolved_acceleration_control(t,x,u,flag,r,L,m,J,Jw)
% Sistema do robô omnidirecional Axebot em espaço de estados.  

% Definir parâmetros
a1 =  2*r^2*m/(2*r^2*m+3*Jw); % não está sendo utilizado (foi parte do cálculo de outras constantes)
a2 = 3*Jw/(2*r^2*m+3*Jw);
b1 = r/(2*r^2*m+3*Jw);
b2 = r*L/(J*r^2+3*Jw*L^2);

switch flag,
case 0,
[sys,x0,str,ts]=mdlInitializeSizes;
  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,a1,a2,b1,b2);

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case {1, 2, 4, 9 },
    sys = [];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end
end
% end csfunc

%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes()
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];
end

function sys=mdlOutputs(t,x,u,a1,a2,b1,b2)
ddxodtt = u(1);
ddyodtt = u(2);
ddphiodtt = u(3);
dxdt = u(4);
dydt = u(5);
phi = u(6);
dphidt = u(7);

beta1 = sin(phi)-sqrt (3)*cos(phi);
beta2 = sin(phi)+sqrt(3)*cos(phi);
beta3 = sqrt(3)*sin(phi)+cos(phi);
beta4 = cos(phi)-sqrt(3)*sin(phi);

X = ddxodtt+a2*dphidt*dydt;
Y = ddyodtt-a2*dphidt*dxdt; 
PHI = ddphiodtt;

u1 = sin(phi)/(6*b1)*X+cos(phi)/(6*b1)*Y+1/(3*b2)*PHI;
u2 = -beta1/(12*b1)*X+beta3/(12*b1)*Y+1/(3*b2)*PHI;
u3 = -beta2/(12*b1)*X+beta4/(12*b1)*Y+1/(3*b2)*PHI;

sys = [u1 u2 u3];
end
% end mdlOutputs
