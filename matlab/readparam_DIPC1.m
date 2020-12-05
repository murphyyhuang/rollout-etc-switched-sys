M = 1.5;
m1 = 0.5;
m2 = 0.75;
l1 = 0.5;
l2 = 0.75;
g = 9.81;

input_dim = 3;
state_dim = 6;
output_dim = 3;

unknown_dim2 = 1;
unknown_dim3 = 1;

% linearized model about the origin
linearA = zeros(6, 6);
linearA(1:3, 4:6) = eye(3);
linearA(4:6, :) =     [ 0, -327/40,         0, 0, 0, 0;
                0,   327/5, -2943/100, 0, 0, 0;
                0, -327/10,    327/10, 0, 0, 0];
            
linearB1 = [0, 0, 0;
    0, 0, 0;
    0, 0, 0;
    2/3,  -4/3,      0;
    -4/3,  32/3,  -16/3;
       0, -16/3, 160/27];

linearC = zeros(output_dim, state_dim);
linearC(1:output_dim, 1:output_dim) = eye(output_dim);
linearD1 = zeros(output_dim, input_dim);

eig(linearA)
disp('Check whether (linearA, linearB1) is controllable or not')
rank(ctrb(linearA,linearB1))  % is it controllable

init_u = [0 0 0]';
init_state_e = [0 0 0 0 0 0]';

% unknown input parameters

% random seed
rng(2333);
% actuator noise: disturbance to the force to the torque 1
% linearB2 = rand(state_dim, unknown_dim2);
linearB2 = rand(6, 1);
% sensor noise: random
linearD2 = linearC * linearB2 * rand(unknown_dim2, unknown_dim3);


%% Optimal control parameters

penalty_alpha = 0.2;
penalty_beta = 1;

mat_Qc = eye(6, 6);
mat_Rc = eye(input_dim);

