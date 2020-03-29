%% Dynamic model of EKF time update step
% x is the curent quartenion/attitude
% P is the state covariance
% omega is the measured angular rate
% T the time since the last measurement
% Rw the process noise covariance matrix
function [x, P] = update_gyro(x, P, T, Rw, omega) 
G = T/2*Sq(x);
% Without angular rate
if nargin < 5
    P = P + G*Rw*G'; 
else % With angular rate
    I = eye(length(x));
        
    F = (I + 1/2*Somega(omega)*T);
    % Discrete time update of a quartenion based on angular velocity omega 
    x = F*x + G*Rw;
    
    % Error Propagation law = Sum(Derivative*Covariance*Derivative')
    P = F*P*F' + G*Rw*G';
end
[x, P] = mu_normalizeQ(x, P);
end