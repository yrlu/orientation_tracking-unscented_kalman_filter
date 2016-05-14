function [q, dcm] = ukf_4(acc_vals, gyro_vals, qd, P, Q, R, ts)
% UKF_4: 4 state UKF
% By Yiren Lu at University of Pennsylvania
% Feb 11 2016
% ESE 650 Project 2

%{ 
Inputs:
    acc_vals, gyro_vals     accelerometer and gyro data (3*n)
    P                       estimate error cov (6*6)
    Q                       process noise cov (6*6)
    R                       measurement noise cov (6*6)
    ts                      time stamp from IMU
Outputs:
    q                       quaternion matrix
    dcm                     rotation matrices (DCMs)
%}




n       = 3;
n_data  = size(acc_vals, 2);
q       = zeros(n_data,4);
dcm    = zeros(3, 3, n_data);
dt  = ts - [ts(1), ts(1 : end - 1)];

thresh  = 1e-3;      
max_iter = 100;



s = [1,0,0,0];   % the initial state in vector space
g = [0,0,0,1];



for i = 1:n_data
    i
    % compute dq from omega and dt
    w = gyro_vals(:,i)';
    w_norm = sqrt(sum(w.^2));
    angle = w_norm*dt(i);
    axis = w/w_norm;
    dq = [cos(angle/2) axis.*sin(angle/2)];
    dq(isnan(dq)) = 0;
    
    
    S = chol(P+Q);          % S 3 * 3
    S = sqrt(2*n)*S;        
    W = [S, -S];            % W 3 * 12
    
    
    X(:, 1:4) = vec2quat(W(1:3,:));
%     X(:, 5:7) = W(4:6,:)';
    X(:, 1:4) = quatmultiply(s(1:4), X(:,1:4));
    
    % Process: Transformations of the sigma points 
    Y(:,1:4) = quatnormalize(quatmultiply(X(:,1:4),dq));
    
    % Computation of the mean with gradient descent
    cur_q = s(1:4);
    err = 1;
    iter = 0;
    
    
    while (err>thresh) && (iter < max_iter) 
        e = quatnormalize(quatmultiply(Y(:,1:4), quatinv(cur_q)));  % e 4*6
        e_vec = quat2vec(e);
        e_mean = mean(e_vec,2);    % 
        err= norm(e_mean);
        cur_q = quatmultiply(vec2quat(e_mean), cur_q);
        iter = iter +1; 
    end
    
    
    x_k(1:4) = cur_q;
    

    % A Priori State Vector Cov
%     Wprime(:,1:3) = e_vec';
    Wprime(:,1:3) = bsxfun(@minus, quat2vec(Y), quat2vec(x_k))'; % works better
    P_k = cov(Wprime);
    
    
    % Measurement: 
    gprime = quatmultiply(Y(:,1:4), quatmultiply(g, quatinv(Y(:,1:4))));
    
    Z(:,1:3) = quat2vec(gprime)'; 
    z_ = mean(Z);
    z = acc_vals(:,i)';
    
    v = z - z_;
    
    
    % Measurement Estimate Cov
    Pzz = cov(Z);
    Pvv = Pzz + R;
    
    
    % Cross correlation matrix
    
    Z_sig = bsxfun(@minus, Z, z_);
    Pxz = 1/2/n * Wprime'*Z_sig;
    
    
    
    % Kalman Gain and Update
    K = Pxz/Pvv;
    P = P_k - K*Pvv*K';
    
    Kv = K*v';
    s(1:4) = quatmultiply(x_k(1:4), vec2quat(Kv(1:3)));
%     s(1:4) = quatmultiply(s(1:4), dq);
%     s(1:4) = x_k(1:4);
    
%     s(1:4)= quatnormalize(quatmultiply(s(1:4), dq));
    q(i,:) = s(1:4);
    dcm(:,:,i) = quat2dcm(q(i,:))';
end
end





function q = vec2quat(vec)
% vec   n*3
% q     n*4
    angle = sqrt(sum(vec(1:3,:).^2,1));
    ev = bsxfun(@rdivide, vec(1:3,:), angle);
    q = [cos(angle/2); bsxfun(@times, ev, sin(angle/2))]';
    q(isnan(q)) = 0;
    q(isinf(q)) = 0;
end

function vec = quat2vec(q)
% vec   n*3
% q     n*4
    q = q';
    vec = (bsxfun(@rdivide, q(2:4,:), sqrt(1 - q(1,:).^2)));
    vec(isnan(vec)) = 0;
    vec(isinf(vec)) = 0;
end


