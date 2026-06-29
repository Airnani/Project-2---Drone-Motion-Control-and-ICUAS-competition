% Unidade Curricular: UAV's
% Project 2 - Drone Motion Control and ICUAS competition

clear all; close all; clc;

% Dados do Drone
m = 0.045;                      
g = 9.81;                       
L = 0.046;                      
l = L * sin(pi/4);              
CQ = 7.75e-11;                  
CT = 3.72e-8;                   
Ctau = CQ/CT;                   
J = diag([2.4e-5, 2.4e-5, 3.2e-5]); 

% Coeficientes de drag
Dx = 0.01; 
Dy = 0.01; 
Dz = 0.01; 
D_mat = diag([Dx, Dy, Dz]);

dark_gray = [0.12, 0.12, 0.12]; 
grid_color = [0.4, 0.4, 0.4];   

% Pergunta 1.4 - Controlador LQR para Estabilização
A = [zeros(3,3), eye(3);
    zeros(3,3), diag([-Dx/m, -Dy/m, -Dz/m])];
B = [zeros(3,3);
    eye(3)];

Q = diag([10, 10, 10, 2, 2, 2]); 
R = diag([1, 1, 1]);              
K = lqr(A, B, Q, R);
disp('Matriz K =');
disp(K);

sys_cl = ss(A - B*K, B, eye(6), 0);
t_estab = 0:0.01:5; 
x0 = [5, -2, -7, 0, 0, 0]; 

[y_estab, t_estab, x_estab] = initial(sys_cl, x0, t_estab);

% Figura 1: Posição Temporal (Estabilização)
fig1 = figure('Color', dark_gray, 'Name', 'Posição - Estabilização 1.4');
ax1 = axes('Parent', fig1, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_estab, x_estab(:, 1), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 2); hold on; 
plot(t_estab, x_estab(:, 2), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2);         
plot(t_estab, x_estab(:, 3), 'Color', [0.93, 0.84, 0.35], 'LineWidth', 2);        
grid on; set(ax1, 'GridColor', grid_color, 'GridAlpha', 0.5);
title('Posição do Drone', 'Color', 'w');
xlabel('Tempo (s)', 'Color', 'w'); ylabel('Posição (m)', 'Color', 'w');
legend({'x', 'y', 'z'}, 'TextColor', 'w', 'Color', dark_gray, 'EdgeColor', 'none');

% Figura 2: Velocidade Temporal (Estabilização)
fig2 = figure('Color', dark_gray, 'Name', 'Velocidade - Estabilização 1.4');
ax2 = axes('Parent', fig2, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_estab, x_estab(:, 4), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 2); hold on; 
plot(t_estab, x_estab(:, 5), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2);         
plot(t_estab, x_estab(:, 6), 'Color', [0.93, 0.84, 0.35], 'LineWidth', 2);        
grid on; set(ax2, 'GridColor', grid_color, 'GridAlpha', 0.5);
title('Velocidade do Drone', 'Color', 'w');
xlabel('Tempo (s)', 'Color', 'w'); ylabel('Velocidade (m/s)', 'Color', 'w');
legend({'vx', 'vy', 'vz'}, 'TextColor', 'w', 'Color', dark_gray, 'EdgeColor', 'none');


% Pergunta 1.6 - Trajetória em 8
dt = 0.01;                        
t_final_8 = 15; 
t_8 = 0:dt:t_final_8;
N_8 = length(t_8);

Ax = 2; Ay = 1; 
omega = 2*pi / 10;   
z_desejado = 2;     

X_8 = [0; 0; 0; 0; 0; 0]; 
hist_x8 = zeros(6, N_8);
hist_xd8_completo = zeros(6, N_8);
erros_posicao = zeros(3, N_8);
comandos_ua   = zeros(3, N_8);

for k = 1:N_8
    hist_x8(:, k) = X_8;
    tk = t_8(k);
    
    pxd = Ax * sin(omega * tk); pyd = Ay * sin(2 * omega * tk); pzd = z_desejado;
    vxd = Ax * omega * cos(omega * tk); vyd = Ay * 2 * omega * cos(2 * omega * tk); vzd = 0;
    axd = -Ax * (omega^2) * sin(omega * tk); ayd = -Ay * ((2 * omega)^2) * sin(2 * omega * tk); azd = 0;
    
    pd = [pxd; pyd; pzd]; vd = [vxd; vyd; vzd]; ad = [axd; ayd; azd];
    Xd = [pd; vd];
    hist_xd8_completo(:, k) = Xd;
    
    % Lei de controlo linear com compensação feedforward
    ua = -K * (X_8 - Xd) + ad + (1/m) * D_mat * vd;
    comandos_ua(:, k) = ua;
    erros_posicao(:, k) = X_8(1:3) - pd;
    
    % Evolução dinâmica do modelo linear
    dXdt = A * X_8 + B * ua;
    X_8 = X_8 + dXdt * dt;
end

% Figura 3: Trajetória 3D
fig3 = figure('Color', dark_gray, 'Name', 'Trajetória 3D (1.6)');
ax3 = axes('Parent', fig3, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
plot3(hist_x8(1,:), hist_x8(2,:), hist_x8(3,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 2.5); hold on;
plot3(hist_x8(1,1), hist_x8(2,1), hist_x8(3,1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); 
plot3(hist_x8(1,end), hist_x8(2,end), hist_x8(3,end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); 
grid on; set(ax3, 'GridColor', grid_color, 'GridAlpha', 0.5);
title('Visualização Tridimensional - Voo em 8', 'Color', 'w');
xlabel('Eixo X (m)', 'Color', 'w'); ylabel('Eixo Y (m)', 'Color', 'w'); zlabel('Eixo Z (m)', 'Color', 'w');
legend({'Percurso Real do Drone', 'Início', 'Fim'}, 'TextColor', 'w', 'Color', dark_gray, 'EdgeColor', 'none', 'Location', 'best');
view(3); axis equal;

% Figura 4: Erros de Posição
fig4_err = figure('Color', dark_gray, 'Name', 'Erros de Posição (1.6)');
ax4_err = axes('Parent', fig4_err, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, erros_posicao(1,:), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 2); hold on;
plot(t_8, erros_posicao(2,:), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2);         
plot(t_8, erros_posicao(3,:), 'Color', [0.93, 0.84, 0.35], 'LineWidth', 2);        
grid on; set(ax4_err, 'GridColor', grid_color, 'GridAlpha', 0.5);
title('Erros de Seguimento de Trajetória Linear', 'Color', 'w');
xlabel('Tempo (s)', 'Color', 'w'); ylabel('Erro de Posição (m)', 'Color', 'w');
legend({'Erro x', 'Erro y', 'Erro z'}, 'TextColor', 'w', 'Color', dark_gray, 'EdgeColor', 'none');
xlim([0 t_final_8]);


% Perguntas 2.1 e 2.2 - Controlador Não-Linear (Lyapunov)
Kp_lyap = diag([5, 5, 5]);  
Kv_lyap = diag([3, 3, 3]);  

X_lyap = [0; 0; 0; 0; 0; 0]; 
hist_lyap = zeros(6, N_8);
ua_lyap_hist = zeros(3, N_8);
atitude_lyap = zeros(2, N_8); 

for k = 1:N_8
    hist_lyap(:, k) = X_lyap;
    tk = t_8(k);
    
    pxd = Ax * sin(omega * tk); pyd = Ay * sin(2 * omega * tk); pzd = z_desejado;
    vxd = Ax * omega * cos(omega * tk); vyd = Ay * 2 * omega * cos(2 * omega * tk); vzd = 0;
    axd = -Ax * (omega^2) * sin(omega * tk); ayd = -Ay * ((2 * omega)^2) * sin(2 * omega * tk); azd = 0;
    
    pd = [pxd; pyd; pzd]; vd = [vxd; vyd; vzd]; ad = [axd; ayd; azd];
    
    ep = X_lyap(1:3) - pd;
    ev = X_lyap(4:6) - vd;
    
    % Lei de controlo não linear
    ua_lyap = ad - Kp_lyap * ep - Kv_lyap * ev + (1/m) * D_mat * X_lyap(4:6);
    ua_lyap_hist(:, k) = ua_lyap; 
    
    % Extração física da atitude 
    uf = ua_lyap + [0; 0; g]; 
    pitch_real = atan2(uf(1), uf(3));
    roll_real  = atan2(-uf(2), sqrt(uf(1)^2 + uf(3)^2));
    atitude_lyap(:, k) = [roll_real; pitch_real];
    
    % Evolução não linear do sistema
    dXdt_lyap = [X_lyap(4:6); ua_lyap];
    X_lyap = X_lyap + dXdt_lyap * dt;
end

% Graficos de comparação e análise
% Figura 5: Comparação Global (Linear vs Lyapunov)
fig5_comp = figure('Color', dark_gray, 'Name', 'Comparação 3D Completa (2.2)');
ax5_comp = axes('Parent', fig5_comp, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
plot3(hist_xd8_completo(1,:), hist_xd8_completo(2,:), hist_xd8_completo(3,:), 'w--', 'LineWidth', 1.5); hold on;
plot3(hist_x8(1,:), hist_x8(2,:), hist_x8(3,:), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 2); 
plot3(hist_lyap(1,:), hist_lyap(2,:), hist_lyap(3,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 2); 
grid on; set(ax5_comp, 'GridColor', grid_color, 'GridAlpha', 0.5);
title('Voo do Drone em 8: LQR vs Lyapunov', 'Color', 'w');
xlabel('Eixo X (m)', 'Color', 'w'); ylabel('Eixo Y (m)', 'Color', 'w'); zlabel('Eixo Z (m)', 'Color', 'w');
legend({'Referência', 'Linear LQR', 'Não Linear Lyapunov'}, 'TextColor', 'w', 'Color', dark_gray, 'EdgeColor', 'none');
view(3); axis equal;

% Figura 6: Erros de posição LQR vs Lyapunov
erro_lin_p = hist_x8(1:3, :) - hist_xd8_completo(1:3, :);
erro_lyap_p = hist_lyap(1:3, :) - hist_xd8_completo(1:3, :);

fig6_errs = figure('Color', dark_gray, 'Name', 'Comparação de Erros (2.2)');
subplot(2,1,1, 'Parent', fig6_errs, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, erro_lin_p(1,:), 'r', t_8, erro_lin_p(2,:), 'g', t_8, erro_lin_p(3,:), 'b', 'LineWidth', 1.5);
grid on; title('Erros de Posição - Controlador Linear LQR', 'Color', 'w'); ylabel('Erro (m)', 'Color', 'w');
legend({'Erro X', 'Erro Y', 'Erro Z'}, 'TextColor', 'w', 'Color', dark_gray);

subplot(2,1,2, 'Parent', fig6_errs, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, erro_lyap_p(1,:), 'r', t_8, erro_lyap_p(2,:), 'g', t_8, erro_lyap_p(3,:), 'b', 'LineWidth', 1.5);
grid on; title('Erros de Posição - Controlador Não Linear Lyapunov', 'Color', 'w');
xlabel('Tempo (s)', 'Color', 'w'); ylabel('Erro (m)', 'Color', 'w');
legend({'Erro X', 'Erro Y', 'Erro Z'}, 'TextColor', 'w', 'Color', dark_gray);

% Figura 7: Atitude Física do Drone
fig7_at = figure('Color', dark_gray, 'Name', 'Atitude Não Linear');
ax7_at = axes('Parent', fig7_at, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, rad2deg(atitude_lyap(1,:)), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2); hold on;
plot(t_8, rad2deg(atitude_lyap(2,:)), 'Color', [0.93, 0.84, 0.35], 'LineWidth', 2);
grid on; set(ax7_at, 'GridColor', grid_color, 'GridAlpha', 0.5);
title('Evolução dos Ângulos de Inclinação (Lyapunov)', 'Color', 'w');
xlabel('Tempo (s)', 'Color', 'w'); ylabel('Ângulos (Graus º)', 'Color', 'w');
legend({'Roll (\phi)', 'Pitch (\theta)'}, 'TextColor', 'w', 'Color', dark_gray, 'EdgeColor', 'none');

% Cálculo de Métricas RMSE
rmse_linear = sqrt(mean(sum(erro_lin_p.^2, 1)));
rmse_lyap = sqrt(mean(sum(erro_lyap_p.^2, 1)));
fprintf(' MÉTRICAS DE DESEMPENHO COMPUTADAS \n');
fprintf('RMSE de Posição - LQR Linear: %.4f metros\n', rmse_linear);
fprintf('RMSE de Posição - Lyapunov Não Linear: %.4f metros\n', rmse_lyap);

% Simulação com rajada de vento
% Definir vento (0.15 Newtons no eixo Y entre os 5s e os 8s)
F_vento_y = zeros(1, N_8);
for k = 1:N_8
    if t_8(k) >= 5 && t_8(k) <= 8
        F_vento_y(k) = 0.15; 
    end
end
a_vento_y = F_vento_y / m; % Aceleração resultante do vento

% Reiniciar estados para a simulação com vento
X_lqr_v = [0; 0; 0; 0; 0; 0];
X_lyap_v = [0; 0; 0; 0; 0; 0];

hist_lqr_v = zeros(6, N_8);
hist_lyap_v = zeros(6, N_8);

% Loop de Simulação Simultâneo com Perturbação
for k = 1:N_8
    hist_lqr_v(:, k) = X_lqr_v;
    hist_lyap_v(:, k) = X_lyap_v;
    tk = t_8(k);

    % Recriar referências da trajetória
    pxd = Ax * sin(omega * tk); pyd = Ay * sin(2 * omega * tk); pzd = z_desejado;
    vxd = Ax * omega * cos(omega * tk); vyd = Ay * 2 * omega * cos(2 * omega * tk); vzd = 0;
    axd = -Ax * (omega^2) * sin(omega * tk); ayd = -Ay * ((2 * omega)^2) * sin(2 * omega * tk); azd = 0;
    Xd = [pxd; pyd; pzd; vxd; vyd; vzd];
    pd = [pxd; pyd; pzd]; vd = [vxd; vyd; vzd]; ad = [axd; ayd; azd];

    % Controlador Linear LQR com vento
    ua_lqr = -K * (X_lqr_v - Xd) + ad + (1/m) * D_mat * vd;
    dX_lqr = A * X_lqr_v + B * ua_lqr + [0; 0; 0; 0; a_vento_y(k); 0];
    X_lqr_v = X_lqr_v + dX_lqr * dt;

    % Controlador Não Linear Lyapunov com vento
    ep = X_lyap_v(1:3) - pd;
    ev = X_lyap_v(4:6) - vd;
    ua_lyap = ad - Kp_lyap * ep - Kv_lyap * ev + (1/m) * D_mat * X_lyap_v(4:6);
    dX_lyap = [X_lyap_v(4:6); ua_lyap + [0; a_vento_y(k); 0]];
    X_lyap_v = X_lyap_v + dX_lyap * dt;
end

% Gráfico de Erro Temporal no Eixo Y (Impacto do vento)
erro_lqr_vy = hist_lqr_v(2, :) - hist_xd8_completo(2, :);
erro_lyap_vy = hist_lyap_v(2, :) - hist_xd8_completo(2, :);

fig_vento = figure('Color', dark_gray, 'Name', 'Análise de Robustez ao Vento');
ax_v = axes('Parent', fig_vento, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, erro_lqr_vy, 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2); hold on;
plot(t_8, erro_lyap_vy, 'Color', [0.0, 1.0, 0.8], 'LineWidth', 2);
grid on; set(ax_v, 'GridColor', grid_color, 'GridAlpha', 0.5);
title('Desvio do Drone no Eixo Y (vento)', 'Color', 'w');
xlabel('Tempo (s)'); ylabel('Erro de Posição Y (m)');
legend({'LQR Linear (Desvia mais)', 'Lyapunov Não Linear (Mais firme)'}, 'TextColor', 'w', 'Color', dark_gray, 'EdgeColor', 'none');

% Display das métricas com vento no Terminal
rmse_lqr_v = sqrt(mean((hist_lqr_v(1:3,:) - hist_xd8_completo(1:3,:)).^2, 'all'));
rmse_lyap_v = sqrt(mean((hist_lyap_v(1:3,:) - hist_xd8_completo(1:3,:)).^2, 'all'));
fprintf('\n DESEMPENHO SOB RAJADA DE VENTO \n');
fprintf('RMSE Global com Vento - LQR Linear: %.4f metros\n', rmse_lqr_v);
fprintf('RMSE Global com Vento - Lyapunov Não Linear: %.4f metros\n', rmse_lyap_v);

% Simulação com rajada de vento e comparação
% Definição da Rajada de Vento (0.15 Newtons no eixo Y entre os 5s e os 8s)
F_vento_y = zeros(1, N_8);
for k = 1:N_8
    if t_8(k) >= 5 && t_8(k) <= 8
        F_vento_y(k) = 0.15; 
    end
end
a_vento_y = F_vento_y / m; % Aceleração resultante do vento (a = F/m)

% Inicialização dos estados para a simulação com perturbação
X_lqr_v = [0; 0; 0; 0; 0; 0];
X_lyap_v = [0; 0; 0; 0; 0; 0];

hist_lqr_v = zeros(6, N_8);
hist_lyap_v = zeros(6, N_8);

% Loop de Simulação com a Perturbação do Vento Ativa
for k = 1:N_8
    hist_lqr_v(:, k) = X_lqr_v;
    hist_lyap_v(:, k) = X_lyap_v;
    tk = t_8(k);
    
    % Recriar referências exatas da trajetória desejada
    pxd = Ax * sin(omega * tk); pyd = Ay * sin(2 * omega * tk); pzd = z_desejado;
    vxd = Ax * omega * cos(omega * tk); vyd = Ay * 2 * omega * cos(2 * omega * tk); vzd = 0;
    axd = -Ax * (omega^2) * sin(omega * tk); ayd = -Ay * ((2 * omega)^2) * sin(2 * omega * tk); azd = 0;
    Xd = [pxd; pyd; pzd; vxd; vyd; vzd];
    pd = [pxd; pyd; pzd]; vd = [vxd; vyd; vzd]; ad = [axd; ayd; azd];
    
    % Dinâmica do Controlador Linear LQR com vento
    ua_lqr = -K * (X_lqr_v - Xd) + ad + (1/m) * D_mat * vd;
    dX_lqr = A * X_lqr_v + B * ua_lqr + [0; 0; 0; 0; a_vento_y(k); 0];
    X_lqr_v = X_lqr_v + dX_lqr * dt;
    
    % Dinâmica do Controlador Não Linear Lyapunov com vento
    ep = X_lyap_v(1:3) - pd;
    ev = X_lyap_v(4:6) - vd;
    ua_lyap = ad - Kp_lyap * ep - Kv_lyap * ev + (1/m) * D_mat * X_lyap_v(4:6);
    dX_lyap = [X_lyap_v(4:6); ua_lyap + [0; a_vento_y(k); 0]];
    X_lyap_v = X_lyap_v + dX_lyap * dt;
end

% Gráficos Comparativos Agrupados de Posições e Velocidades
% Figura extra A: Comparaçãao das Posições (X, Y, Z)
fig_pos_comp = figure('Color', dark_gray, 'Name', 'Comparação de Posições com Vento');

% Subplot Eixo X
subplot(3, 1, 1, 'Parent', fig_pos_comp, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(1,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_lqr_v(1,:), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 1.8);
plot(t_8, hist_lyap_v(1,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; ylabel('Posição X (m)', 'Color', 'w');
title('Comparação de Posições com Rajada de Vento', 'Color', 'w');
legend({'Ref', 'LQR Linear', 'Lyapunov'}, 'TextColor', 'w', 'Color', dark_gray);

% Subplot Eixo Y (Onde o vento atua diretamente)
subplot(3, 1, 2, 'Parent', fig_pos_comp, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(2,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_lqr_v(2,:), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 1.8);
plot(t_8, hist_lyap_v(2,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; ylabel('Posição Y (m)', 'Color', 'w');

% Subplot Eixo Z
subplot(3, 1, 3, 'Parent', fig_pos_comp, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(3,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_lqr_v(3,:), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 1.8);
plot(t_8, hist_lyap_v(3,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; xlabel('Tempo (s)', 'Color', 'w'); ylabel('Posição Z (m)', 'Color', 'w');

% Figura Extra B: Comparação Das Velocidades (VX, VY, VZ)
fig_vel_comp = figure('Color', dark_gray, 'Name', 'Comparação de Velocidades com Vento');

% Subplot Velocidade X
subplot(3, 1, 1, 'Parent', fig_vel_comp, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(4,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_lqr_v(4,:), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 1.8);
plot(t_8, hist_lyap_v(4,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; ylabel('Velocidade Vx (m/s)', 'Color', 'w');
title('Comparação de Velocidades com Rajada de Vento', 'Color', 'w');
legend({'Ref', 'LQR Linear', 'Lyapunov'}, 'TextColor', 'w', 'Color', dark_gray);

% Subplot Velocidade Y
subplot(3, 1, 2, 'Parent', fig_vel_comp, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(5,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_lqr_v(5,:), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 1.8);
plot(t_8, hist_lyap_v(5,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; ylabel('Velocidade Vy (m/s)', 'Color', 'w');

% Subplot Velocidade Z
subplot(3, 1, 3, 'Parent', fig_vel_comp, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(6,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_lqr_v(6,:), 'Color', [0.85, 0.33, 0.1], 'LineWidth', 1.8);
plot(t_8, hist_lyap_v(6,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; xlabel('Tempo (s)', 'Color', 'w'); ylabel('Velocidade Vz (m/s)', 'Color', 'w');

% Output de Dados RMSE para Validação das Figuras
rmse_lqr_v = sqrt(mean((hist_lqr_v(1:3,:) - hist_xd8_completo(1:3,:)).^2, 'all'));
rmse_lyap_v = sqrt(mean((hist_lyap_v(1:3,:) - hist_xd8_completo(1:3,:)).^2, 'all'));
fprintf('\n ANÁLISE DE PERFORMANCE GLOBAL COM VENTO \n');
fprintf('LQR Linear - Posição Global RMSE: %.4f metros\n', rmse_lqr_v);
fprintf('Lyapunov Não Linear - Posição Global RMSE: %.4f metros\n', rmse_lyap_v);

% Gráficos comparativos agrupados (Sem vento)
% Figura Extra C: Comparação das Posições (X, Y, Z) - Sem Vento
fig_pos_sv = figure('Color', dark_gray, 'Name', 'Comparação de Posições (Sem Vento)');

% Subplot Posição X
subplot(3, 1, 1, 'Parent', fig_pos_sv, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(1,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_x8(1,:), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.8); % Azul para LQR
plot(t_8, hist_lyap(1,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8); % Ciano para Lyapunov
grid on; ylabel('Posição X (m)', 'Color', 'w');
title('Comparação de Posições (Cenário Sem Vento)', 'Color', 'w');
legend({'Ref', 'LQR Linear', 'Lyapunov'}, 'TextColor', 'w', 'Color', dark_gray, 'EdgeColor', 'none');

% Subplot Posição Y
subplot(3, 1, 2, 'Parent', fig_pos_sv, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(2,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_x8(2,:), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.8);
plot(t_8, hist_lyap(2,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; ylabel('Posição Y (m)', 'Color', 'w');

% Subplot Posição Z
subplot(3, 1, 3, 'Parent', fig_pos_sv, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(3,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_x8(3,:), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.8);
plot(t_8, hist_lyap(3,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; xlabel('Tempo (s)', 'Color', 'w'); ylabel('Posição Z (m)', 'Color', 'w');

% Figura Extra D: Comparação das Velocidades (VX, VY, VZ) - Sem Vento
fig_vel_sv = figure('Color', dark_gray, 'Name', 'Comparação de Velocidades (Sem Vento)');

% Subplot Velocidade Vx
subplot(3, 1, 1, 'Parent', fig_vel_sv, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(4,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_x8(4,:), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.8);
plot(t_8, hist_lyap(4,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; ylabel('Velocidade Vx (m/s)', 'Color', 'w');
title('Comparação de Velocidades (Cenário Sem Vento)', 'Color', 'w');
legend({'Ref', 'LQR Linear', 'Lyapunov'}, 'TextColor', 'w', 'Color', dark_gray, 'EdgeColor', 'none');

% Subplot Velocidade Vy
subplot(3, 1, 2, 'Parent', fig_vel_sv, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(5,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_x8(5,:), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.8);
plot(t_8, hist_lyap(5,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; ylabel('Velocidade Vy (m/s)', 'Color', 'w');

% Subplot Velocidade Vz
subplot(3, 1, 3, 'Parent', fig_vel_sv, 'Color', dark_gray, 'XColor', 'w', 'YColor', 'w');
plot(t_8, hist_xd8_completo(6,:), 'w--', 'LineWidth', 1.2); hold on;
plot(t_8, hist_x8(6,:), 'Color', [0.0, 0.45, 0.74], 'LineWidth', 1.8);
plot(t_8, hist_lyap(6,:), 'Color', [0.0, 1.0, 0.8], 'LineWidth', 1.8);
grid on; xlabel('Tempo (s)', 'Color', 'w'); ylabel('Velocidade Vz (m/s)', 'Color', 'w');

Acl = A - B*K;
lambda = eig(Acl);
fprintf('\n Valores próprios do sistema em malha fechada: \n')
disp(lambda)