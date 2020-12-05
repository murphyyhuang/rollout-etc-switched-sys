%% System linearization at the origin

clear; clc;

readparam_DIPC1; % parameters that linearized at the origin


if ~exist('flag_anima', 'var')
    flag_anima = 0;
end

if ~exist('video_flag', 'var')
    video_flag = 1;
end

if ~exist('period_t', 'var')
    period_t = 0.001;
end

if ~exist('period_tau', 'var')
    period_tau = 0.1;
end

if ~exist('var_q', 'var')
    var_q = 3;
end

if ~exist('alpha_c', 'var')
    alpha_c = 0;
end

if ~exist('end_time', 'var')
    end_time = 20;
end

if ~exist('init_state', 'var')
    init_state = init_state_e;
    init_state = init_state + [0, pi/6, pi/6, 0, 0, 0]';
end

if ~exist('video_name', 'var')
    video_name = ['../avi/periodic_ctrl_tau', num2str(period_tau), '_param1.avi'];
end

if ~exist('load_flag', 'var')
    load_flag = false;
end


% discretization of C-T LTI Systems
[mat_A_qtau, mat_B_qtau, ~, ~] = ...
    c2dm(linearA, linearB1, linearC, zeros(size(linearC, 1), size(linearB1, 2)), var_q * period_tau, 'zoh');

% discretization of dicounted cost function

if  ~load_flag
    syms s real
    augsys_mat = [linearA, linearB1; zeros(input_dim, state_dim), zeros(input_dim, input_dim)];
    % quad_integrand = exp(-alpha_c * s) * expm(vpa(augsys_mat))'  ...
    %     * [mat_Qc, zeros(state_dim, input_dim); zeros(input_dim, state_dim), mat_Rc] ...
    %     * expm(vpa(augsys_mat));
    approx_expm_augsys = eye(input_dim + state_dim) + s * augsys_mat ...
        + 1/2 * s^2 * augsys_mat^2 + 1/6 * s^3 * augsys_mat^3 ...
        + 1/24 * s^4 * augsys_mat^4 + 1/120 * s^5 * augsys_mat^5 ...
        + 1/720 * s^6 * augsys_mat^6;

    integrand = exp(-alpha_c * s) * approx_expm_augsys'  ...
        * blkdiag(penalty_alpha * mat_Qc,  penalty_beta * mat_Rc) ...
        * approx_expm_augsys;

    quad_mat = int(...
        integrand, ...
        [0, period_tau * var_q]);
    quad_mat = double(quad_mat);

    mat_Q_qtau = quad_mat(1:state_dim, 1:state_dim);
    mat_S_qtau = quad_mat(1:state_dim, state_dim + 1:state_dim + input_dim);
    mat_R_qtau = quad_mat(state_dim + 1:state_dim + input_dim, state_dim + 1:state_dim + input_dim);

    save(['periodic_ctrl_tau', num2str(period_tau), '_q', num2str(var_q), '.mat'], ...
        'mat_Q_qtau', 'mat_S_qtau', 'mat_R_qtau');
else
    
    load(['periodic_ctrl_tau', num2str(period_tau), '_q', num2str(var_q), '.mat']);

end


alpha_qtau = exp(-alpha_c * period_tau * var_q);


%% Optimal Stationary Policy

mat_R_qtau_scale = 1 / alpha_qtau * mat_R_qtau;
mat_A_qtau_scale = sqrt(alpha_qtau) * mat_A_qtau;
mat_S_qtau_scale = 1 / sqrt(alpha_qtau) * mat_S_qtau;

[mat_P_qtau, mat_K_qtau, L_qtau, dare_info] = idare( ...
    mat_A_qtau_scale, mat_B_qtau, mat_Q_qtau, mat_R_qtau_scale, mat_S_qtau_scale, [] ... 
);
% mat_K_qtau = -inv(mat_R_qtau + alpha_qtau * mat_B_qtau' * mat_P_qtau * mat_B_qtau)  ...
%     * (alpha_qtau * mat_B_qtau' * mat_P_qtau * mat_A_qtau + mat_S_qtau');

mat_K_qtau = 1 / sqrt(alpha_qtau) * mat_K_qtau;
% eig(mat_A_qtau - mat_B_qtau * mat_K_qtau)


%% Simulation of the System

obsv_time = 0:period_t:end_time;

x_real_array = zeros(state_dim, size(obsv_time, 2));
u_array = zeros(input_dim, size(obsv_time, 2));
x_real_array(:, 1) = init_state;
u_array(:, 1) = init_u;

step_size = (period_tau * var_q) / period_t;
u1 = init_u;

for t_index = 1:size(obsv_time, 2)-1
    
    t_real = obsv_time(t_index);
    
    % control policy
    if ~rem(t_index - 1, step_size)
        u1 = - (mat_K_qtau * x_real_array(:, t_index) - init_u);
    end
    
    % nonlinear physical system
    dX_real = ode_3dinputDIPC( ...
        x_real_array(:, t_index), u1, M, m1, m2, l1, l2, g);
    
    x_real_array(:, t_index + 1) = x_real_array(:, t_index) + dX_real * period_t;
    u_array(:, t_index + 1) = u1;
    
end

figure()
if flag_anima

    if video_flag
        dipcVideo = VideoWriter(video_name);
        dipcVideo.FrameRate = 10;
        open(dipcVideo)
    end
    
    animaHandler = initializeDIPC(x_real_array(:, 1), M, m1, m2, l1, l2);
    % Animation
    for k = 1:fix(length(obsv_time) / 5)
        updateDIPC(x_real_array(:, k), l1, l2, animaHandler, obsv_time(k));
        if video_flag
            writeVideo(dipcVideo,getframe(gca));
        end
        
    end
    
    if video_flag
        close(dipcVideo);
    end
end


figure();
for plot_i = 1:state_dim
    subplot(3, 2, plot_i);
    plot(obsv_time, x_real_array(plot_i, :)', '-');
    hold on;
    xlabel('Time (seconds)');
    set(get(gca,'YLabel'),'Rotation',0);
    grid on;
    legend(['$x_', num2str(plot_i), '$'],  ...
        'Interpreter','latex');
end

save(['./tmp/periodic_ctrl_simu_tau', num2str(period_tau), '_q', num2str(var_q), '.mat'], ...
    'x_real_array', 'u_array');
