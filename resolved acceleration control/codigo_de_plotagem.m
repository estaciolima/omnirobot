% velocidade
figure(1);
subplot(2,2,1);
plot(dxdt_ref.time, dxdt_ref.data, '--', velocidade_x.time, velocidade_x.data, 'b','LineWidth',1.2);
title('(a)');
legend('Setpoint', 'Real');
xlabel('Tempo (s)');
ylabel('Velocidade eixo x (m/s)');
axis tight;
grid on;

subplot(2,2,2);
plot(dydt_ref.time, dydt_ref.data, '--', velocidade_y.time, velocidade_y.data, 'b','LineWidth',1.2);
title('(b)');
legend('Setpoint', 'Real');
xlabel('Tempo (s)');
ylabel('Velocidade eixo y (m/s)');
axis tight;
grid on;

% trajetória
subplot(2,2,3);
plot(x_ref.data(1:250:end), y_ref.data(1:250:end),'o', x.data(1:250:end), y.data(1:250:end), 'bx'); 
title('(c)');
legend('Setpoint', 'Real','Location','Best');
xlabel('x (m)');
ylabel('y (m)');
axis tight;
grid on;

% erro de trajetória (distância euclidiana)
subplot(2,2,4);
erro_euclidiano = sqrt((x_ref.data-x.data).^2+(y_ref.data-y.data).^2);
plot(x.time, erro_euclidiano,'b', 'LineWidth', 1.2);
title('(d)');
xlabel('Tempo (s)');
ylabel('Erro (m)');
axis tight;
grid on;

% orientação
figure(2);
plot(phi_ref.time(1:250:end), phi_ref.data(1:250:end), '--', phi.time(1:250:end), phi.data(1:250:end), 'b', 'LineWidth', 1.2);
legend('Setpoint', 'Real');
xlabel('Tempo (s)');
ylabel('Orientação (rad)');
axis tight;
grid on;
% Torques
figure(3);
plot(torque1.time, torque1.data, torque2.time, torque2.data, torque3.time, torque3.data, 'LineWidth', 1.2)
xlabel('Tempo (s)');
ylabel('Torque (Nm)');
legend('Roda 1', 'Roda 2', 'Roda 3');
axis tight;
grid on;

% Tensões do motor
figure(4);
plot(tensao_motor1.time, tensao_motor1.data, tensao_motor2.time, tensao_motor2.data, tensao_motor3.time, tensao_motor3.data, 'LineWidth', 1.2)
xlabel('Tempo (s)');
ylabel('Voltagem (V)');
legend('Motor 1', 'Motor 2', 'Motor 3');
axis tight;
grid on;

% Diferença entre torque estimado e real
figure(5);
plot(torque3_estimado.time, torque3_estimado.data, torque3_real.time, torque3_real.data);
xlabel('Tempo (s)');
ylabel('Torque (Nm)');
legend('Estimado', 'Real');
grid on;

% Erro do torque estimado
figure(6);
erro = abs(torque3_real.data-torque3_estimado.data);
plot(torque3_estimado.time(100:end), erro(100:end));
xlabel('Tempo (s)');
ylabel('Erro absoluto (Nm)');
grid on;

