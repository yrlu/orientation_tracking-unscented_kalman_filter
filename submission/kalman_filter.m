% Basic Linear Kalman Filter
% By Yiren Lu at UPenn, Feb 9, 2016
% ESE Project 2
% Inputs:
%       x           prior state
%       sigma       covariance matrix of prior distribution
%       u           the control input
%       z           the observation
%       p_noise     covariance matrix of the process gaussian noise
%       m_noise     covariance matrix of the measurement gaussian noise
%       A           state transformation matrix
%       B           control matrix
%       C           measurement matrix

% Outputs: 
%       x           posterior state
%       sigma       covariance matrix of the posterior distribution
%

function [x, sigma] = kalman_filter(x, sigma, u, z, p_noise, m_noise, A, B, C)

x = A*x + B*u;
sigma = A*sigma*A' + p_noise;
K = sigma*C'*inv(C*sigma*C'+m_noise);
x = x + K*(z - C*x);
sigma = (eye(size(K,1)) - K*C)*sigma;

end
