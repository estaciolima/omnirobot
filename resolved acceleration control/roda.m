function [sys,x0,str,ts] = roda(t,x,u,flag,Vd,Vsat,deltaVd,L,Kem,R,Kti,Jm,eta,N)
% Sistema do robô omnidirecional Axebot em espaço de estados.  

% Definir parâmetros
Vd_real = Vd-2*Vsat-deltaVd;

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
    sys=mdlDerivatives(t,x,u,L,Vd_real,Kem,R,N);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u,eta,N,Kti,Jm);

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
sizes.NumContStates  = 1;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;

sys = simsizes(sizes);
x0  = 0;
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
function sys=mdlDerivatives(t,x,u,L,Vd_real,Kem,R,N)
i = x(1);
delta = u(1);
Ww = u(2);
Wm = N*Ww;
% Wm = Ww;

didt = 1/L*(delta*Vd_real-Kem*Wm-R*i);

sys = didt;
end
% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,eta,N,Kti,Jm)
i = x(1);
dWmdt = u(3)*N;
% dWmdt = u(3);

Tw = eta*N*(Kti*i-Jm*dWmdt);

sys = [i Tw];
end

% end mdlOutputs
