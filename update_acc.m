function [x, P] = update_acc(x, P, acc, Rg, g_base)
    % Measurement estimate
    hx = Qq(x)'*g_base;
    
    % Derivatives of Q
    [Q0, Q1, Q2, Q3] = dQqdq(x);
    
    % Calculate Jacobian matrix 
    Hx = [Q0'*g_base Q1'*g_base Q2'*g_base Q3'*g_base];
    
    % Calculate Innovation covariance and Kalman gain
    Sk = Hx*P*Hx'+Rg;
    Kk = P*Hx'/Sk;
    
    % Update x and P
    x = x + Kk*(acc-hx);
    P = P-Kk*Sk*Kk';  
    
    [x, P] = mu_normalizeQ(x,P);
end