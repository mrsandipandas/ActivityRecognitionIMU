%% Dynamic model of EKF time update step
% x is the curent quartenion/attitude
% P is the state covariance
% omega is the measured angular rate
% T the time since the last measurement
% Rw the process noise covariance matrix
function [x, P] = update_gyro(x, P, T, Rw, omega) 
I = eye(length(x));
% Without angular rate
if nargin < 5
    F = I;
else % With angular rate
    F = (I + T/2*Somega(omega));
end
G = T/2*Sq(x);
    
% Discrete time update of a quartenion based on angular velocity omega 
x = F*x;

% Error Propagation law = Sum(Derivative*Covariance*Derivative')
P = F*P*F' + G*Rw*G';

[x, P] = mu_normalizeQ(x, P);
end