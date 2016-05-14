function [q, dcm] = ukf_7(acc_vals, gyro_vals, qd, P, Q, R, ts)
% UKF_7 7 state UKF

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


n       = 6;
n_data  = size(acc_vals, 2);
q       = zeros(n_data,4);
dcm    = zeros(3, 3, n_data);
dt  = ts - [ts(1), ts(1 : end - 1)];
% dt = ones(size(ts,1), size(ts,2))*0.01;
thresh  = 1e-3;      
max_iter = 100;


s = [1,0,0,0,0,0,0];   % the initial state in vector space
g = [0,0,0,1];




for i = 1:n_data
    i
    % compute dq from omega and dt
%     w = gyro_vals(:,i)';
    w= s(5:7);
    w_norm = sqrt(sum(w.^2));
    angle = w_norm*dt(i);
    axis = w/w_norm;
    dq = [cos(angle/2) axis.*sin(angle/2)];
    dq(isnan(dq)) = 0;
    
    % compute sigma points
    
    S = chol(P+Q);          % S 6 * 6
    S = sqrt(2*n)*S;        
    W = [S, -S];            % W 6 * 12
    
    % convert 6d W into 7d sigma points X: 12*7
    X(:, 1:4) = vec2quat(W(1:3,:));
    X(:, 5:7) = W(4:6,:)';
    X(:, 1:4) = quatmultiply(s(1:4), X(:,1:4));
    

    % Process: Transformations of the sigma points 
%     Y(:,1:4) = quatnormalize(quatmultiply(X(:,1:4),dq));
    Y(:,1:4) = quatmultiply(X(:,1:4),dq);
    Y(:,5:7) = bsxfun(@plus, X(:,5:7), w);
    
    % Computation of the mean with gradient descent
%     cur_q = s(1:4);
%     err = 1;
%     iter = 0;
%     
%     while (err>thresh) && (iter < max_iter) 
%         e = quatmultiply(Y(:,1:4), quatconj(cur_q));  % e 4*6
%         e_vec = quat2vec(e);
%         e_mean = mean(e_vec,2);    % 
%         err= norm(e_mean);
%         cur_q = quatmultiply(vec2quat(e_mean), cur_q);
%         iter = iter +1; 
%     end
    [cur_q2]=avg_quaternion_markley(Y(:,1:4));
%     cur_q2
    x_k(1:4) = cur_q2;
    x_k(5:7) = mean(Y(:,5:7), 1);
    
%     Wprime(:,1:3) = e_vec';
%     Wprime(:,1:3) = bsxfun(@minus, quat2vec(Y(:,1:4)), quat2vec(x_k(1:4)))';
    Wprime(:,1:3) = quat2vec(quatmultiply(Y(:,1:4), quatconj(x_k(1:4))))';
    Wprime(:,4:6) = bsxfun(@minus, Y(:, 5:7), x_k(5:7));
    P_k = Wprime'*Wprime/2/n;
%     P_k = cov(Wprime);
    
    
    % Measurement: 
%     gprime = quatmultiply(quatmultiply(quatconj(Y(:,1:4)), g), Y(:,1:4));
    gprime = quatmultiply(Y(:,1:4), quatmultiply(g, quatconj(Y(:,1:4))));
    Z(:,1:3) = gprime(:,2:4); 
    Z(:,4:6) = Y(:,5:7);
    
    z_ = mean(Z);
    z = [acc_vals(:,i); gyro_vals(:,i)]';

    v = z - z_;
    
    % Measurement Estimate Cov
    Wz = bsxfun(@minus, Z, z_);
    Pzz = Wz'*Wz/2/n;
    Pvv = Pzz + R;
    
    % Cross correlation matrix
    
    Z_sig = bsxfun(@minus, Z, z_);
    Pxz = 1/2/n * Wprime'*Z_sig;
    
    % Kalman Gain and Update
    K = Pxz/Pvv;
    P = P_k - K*Pvv*K';
    Kv = K*v';
    
    s(1:4) = quatmultiply(x_k(1:4), vec2quat(Kv(1:3)));
    s(5:7) = x_k(5:7) + Kv(4:6)';
    
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
    
    q = quatnormalize(q)';
    angles = acos(q(1,:))*2; % 1*n;
    sins = sin(angles); % 1*n
    vec = bsxfun(@times, (bsxfun(@rdivide, q(2:4,:), sins))', sins')';
%     vec = bsxfun(@times, (bsxfun(@rdivide, q(2:4,:), sqrt(1 - q(1,:).^2)))', acos(q(1,:))'*2)';
    vec(isnan(vec)) = 0;
    vec(isinf(vec)) = 0;
end