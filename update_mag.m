function [x, P] = mu_mag(x, P, mag, m_base, Rm)
    % MU_M EKF update using magnetometer measurements

    % Calculate measurement estimate
    hx = Qq(x)'*m_base;
                        
    % Get derivatives of Q
    [Q0, Q1, Q2, Q3] = dQqdq(x);
    
    % Calculate Jacobian matrix 
    Hx = [Q0'*m_base Q1'*m_base Q2'*m_base Q3'*m_base];
    
    % Calculate covariance propagation and Kalman gain
    Sk = Hx*P*Hx'+Rm;
    Kk = P*Hx'/Sk;
    
    % Update x and P
    x = x + Kk*(mag-hx);
    P = P-Kk*Sk*Kk';
    
    [x, P] = mu_normalizeQ(x,P);
end