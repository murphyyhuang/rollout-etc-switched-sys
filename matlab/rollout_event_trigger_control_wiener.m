%% System linearization at the origin

clear; clc;

readparam_DIPC1; % parameters that linearized at the origin


if ~exist('flag_anima', 'var')
    flag_anima = 1;
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

if ~exist('h_step', 'var')
    h_step = 15;
end

if ~exist('var_q', 'var')
    var_q = 3;
end

if ~exist('alpha_c', 'var')
    alpha_c = 0.5;
end

if ~exist('end_time', 'var')
    end_time = 20;
end

if ~exist('noise_magnit', 'var')
    noise_magnit = 0.1;
end

if ~exist('init_state', 'var')
    init_state = init_state_e;
    init_state = init_state + [0, pi/6, pi/6, 0, 0, 0]';
end

if ~exist('video_name', 'var')
    video_name = ['../avi/rollout_et_ctrl_tau', num2str(period_tau), ...
        '_q', num2str(var_q), '_param1.avi'];
end

if ~exist('load_flag', 'var')
    load_flag = true;
end


%% Discretization of C-T LTI Systems
[mat_A_tau, mat_B_tau, ~, ~] = ...
    c2dm(linearA, linearB1, [], [],  period_tau, 'zoh');

mat_A0 = [mat_A_tau, mat_B_tau; zeros(input_dim, state_dim), eye(input_dim)];
mat_B0 = zeros(state_dim + input_dim, input_dim);

mat_A1 = blkdiag(mat_A_tau, zeros(input_dim));
mat_B1 = [mat_B_tau; eye(input_dim)];

[mat_A_qtau, mat_B_qtau, ~, ~] = ...
    c2dm(linearA, linearB1, [], [],  period_tau * var_q, 'zoh');

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

    % time step: 0, ..., h-1
    quad_mat_tau = int(...
        integrand, ...
        [0, period_tau]);
    quad_mat_tau = double(quad_mat_tau);

    mat_Q_tau = quad_mat_tau(1:state_dim, 1:state_dim);
    mat_S_tau = quad_mat_tau(1:state_dim, state_dim + 1:state_dim + input_dim);
    mat_R_tau = quad_mat_tau(state_dim + 1:state_dim + input_dim, state_dim + 1:state_dim + input_dim);

    mat_Q0 = quad_mat_tau;
    mat_Q1 = blkdiag(mat_Q_tau, zeros(input_dim));
    mat_S1 = [mat_S_tau; zeros(input_dim)];
    mat_R1 = mat_R_tau;
    
    % time step: h, ..., 2h - 1, 2h, ...
    quad_mat_qtau = int(...
        integrand, ...
        [0, period_tau * var_q]);
    quad_mat_qtau = double(quad_mat_qtau);

    mat_Q_qtau = quad_mat_qtau(1:state_dim, 1:state_dim);
    mat_S_qtau = quad_mat_qtau(1:state_dim, state_dim + 1:state_dim + input_dim);
    mat_R_qtau = quad_mat_qtau(state_dim + 1:state_dim + input_dim, state_dim + 1:state_dim + input_dim);
    
    % discretization of Gaussian white noise

    wienerIntg = eye(state_dim) + s * linearA ...
            + 1/2 * s^2 * linearA^2 + 1/6 * s^3 * linearA^3 ...
            + 1/24 * s^4 * linearA^4 + 1/120 * s^5 * linearA^5 ...
            + 1/720 * s^6 * linearA^6 + 1/5040 * s^7 * linearA^7;
    wienerIntg = wienerIntg * noise_magnit * eye(state_dim);
    wienerIntg = wienerIntg * wienerIntg';

    Phi_tau =  int(...
            wienerIntg, ...
            [0, period_tau]);
    Phi_tau = double(Phi_tau);
    Phi_tau = blkdiag(Phi_tau, zeros(input_dim));
    
    Phi_qtau =  int(...
        wienerIntg, ...
        [0, period_tau * var_q]);
    Phi_qtau = double(Phi_qtau);
    
    save(['rollout_ctrl_tau', num2str(period_tau), '_q', num2str(var_q), '.mat'], ...
        'mat_Q_tau', 'mat_S_tau', 'mat_R_tau', ...
        'mat_Q_qtau', 'mat_S_qtau', 'mat_R_qtau', ...
        'mat_Q0', 'mat_Q1', 'mat_S1', 'mat_R1', ...
        'Phi_tau', 'Phi_qtau');
else
    
    load(['rollout_ctrl_tau', num2str(period_tau), '_q', num2str(var_q), '.mat']);

end


alpha_tau = exp(-alpha_c * period_tau);
alpha_qtau = exp(-alpha_c * period_tau * var_q);


%% Optimal Periodic Stationary Policy

mat_R_qtau_scale = 1 / alpha_qtau * mat_R_qtau;
mat_A_qtau_scale = sqrt(alpha_qtau) * mat_A_qtau;
mat_S_qtau_scale = 1 / sqrt(alpha_qtau) * mat_S_qtau;

[mat_P_qtau, mat_K_qtau, L_qtau, dare_info] = idare( ...
    mat_A_qtau_scale, mat_B_qtau, mat_Q_qtau, mat_R_qtau_scale, mat_S_qtau_scale, [] ... 
);

mat_K_qtau = 1 / sqrt(alpha_qtau) * mat_K_qtau;
% eig(mat_A_qtau - mat_B_qtau * mat_K_qtau)


%% Scheduling Sequence

schedule_indices = nchoosek(1:h_step, h_step/var_q);
binary_schedules = zeros(size(schedule_indices, 1), h_step);
nSches = size(schedule_indices, 1);
for iSche = 1: nSches
    schedule_indice = schedule_indices(iSche, :);
    binary_schedules(iSche, schedule_indice) = 1;
end

% calculate the backward recursion matrix

PList{nSches} = [];
WList{nSches} = [];
KList{nSches} = [];

Whi = blkdiag(mat_P_qtau, zeros(input_dim));

for iSche = 1: nSches
    
    binary_schedule = binary_schedules(iSche, :);
    [P_i, Wi_list, Ki_list] = switched_riccati_map(...
        Whi, alpha_tau, mat_A0, mat_Q0, ...
        mat_A1, mat_Q1, mat_S1, mat_B1, mat_R1, ...
        binary_schedule, state_dim, input_dim ...
    );

    PList{iSche} = P_i;
    WList{iSche} = Wi_list;
    KList{iSche} = Ki_list;
    
end


%% Simulation of the System

obsv_time = 0:period_t:end_time;

x_real_array = zeros(state_dim, size(obsv_time, 2));
u_array = zeros(input_dim, size(obsv_time, 2));
x_real_array(:, 1) = init_state;
u_array(:, 1) = init_u;

schedule_period = (period_tau * h_step) / period_t;
ctrl_period = period_tau / period_t;
u1 = init_u;
KListChosen = {};
minSches = [];

for t_index = 1:size(obsv_time, 2)-1
    
    t_real = obsv_time(t_index);
    x_tmp = x_real_array(:, t_index);
    xi_tmp = [x_tmp; u1];
    
    % control policy
    if ~rem(t_index - 1, schedule_period)

        cost_values = [];
  
        for iSche = 1:nSches
            
            cost_value = xi_tmp' * PList{iSche} * xi_tmp;
            
            noise_cost = 0;
            for iSeq = 1:h_step
                noise_cost = noise_cost + ...
                    alpha_tau ^ iSeq * trace(WList{iSche}{iSeq + 1} * Phi_tau);
            end
            
            cost_value = cost_value + noise_cost;
            cost_values = [cost_values, cost_value];
        end
        
        [mincostVal, minSche] = min(cost_values);
        KListChosen = KList{minSche};
        minSches = [minSches, minSche];
        
    end
    if ~rem(t_index - 1, ctrl_period)
        iInner = rem((t_index - 1) / ctrl_period, h_step) + 1;
        mat_K = KListChosen{iInner};
        
        if mat_K ~= false
            u1 = -(mat_K * x_tmp - init_u);
        end
        
    end
        
    % nonlinear physical system
    dX_real = ode_3dinputDIPC_wiener( ...
        x_real_array(:, t_index), u1, M, m1, m2, l1, l2, g, noise_magnit);
    
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
    for k = 1:20:fix(length(obsv_time) / 2)
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

save(['./tmp/rollout_wiener_ctrl_simu_tau', num2str(period_tau), '_q', num2str(var_q), '.mat'], ...
    'x_real_array', 'u_array');
