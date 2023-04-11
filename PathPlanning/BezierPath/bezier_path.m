% https://tajimarobotics.com/bezier-curve-interpolation/
% https://1st-archiengineer.com/programing-bspline2/
function [] = bezier_path()
    clear;
    close all;
    clc;
    
    % Control Points (X-Y)
%     P = [ 
%         0.0, 0.0;
%         1.0, 2.0;
%         3.0, 2.0;
%         3.0, 0.0;
%         5.0, 2.0;
%         6.0, 0.0;
%         ];
    
    P = [ 3, 3;
          6, 1;
          9, 1;
          9, 5;
          12, 5;
          16.5, 9.5;
          9, 13;
          6, 8];
  
    p = length(P);  % Number of Control Ponits
    n = p - 1;      % Degree of B-Spline
    m = p + n + 1;  % Number of Knots in Knot Vector

    % Define Knot Vector
    u = OpenUniformKnotVector(m,n);

    t = 0:0.01:u(end);

    % Calculate B-spline Curve
    S = zeros(length(t),2);
    S(1,:) = P(1,:);
    for i = 2:length(t)
        for j =1:p
            b = BasisFunction(u,j,n,t(i));  
            S(i,:) = S(i,:) + P(j,:)*b;
        end
    end
    
    figure;
    plot(P(:, 1), P(:, 2), '-o'); hold on;
    plot(S(:, 1), S(:, 2), 'LineWidth', 2); hold on;
end

% u : Knot Vector
% m : Number of Knots in Knot Vector
% n : Degree of B-Spline
function u = OpenUniformKnotVector(m,n)
    u = zeros(1,m);
    for i = 1:1:m
          if i < n+1 
              u(i) = 0;
          elseif i > m - (n + 1) 
              u(i) = m - 1 - 2 * n;
          else
              u(i) = i - n - 1;
          end
    end
    u = u/u(end);      
end

% u : Knot Vector
% j : j-th Control Ponit
% k : k-th Basis Function <-- n-th B-Spline
% t : Time
function var = BasisFunction(u,j,k,t)   
    w1 = 0.0;
    w2 = 0.0;
    
    if k == 0 
        if u(j) < t && t <= u(j+1)
            var = 1.0;
        else
            var = 0.0;
        end
    else
        if (u(j+k+1)-u(j+1)) ~= 0  
            w1 = BasisFunction(u,j+1,k-1,t) * (u(j+k+1) - t)/(u(j+k+1)-u(j+1));
        end
        if (u(j+k)-u(j)) ~= 0  
            w2 = BasisFunction(u,j,k-1,t) * (t - u(j))/(u(j+k) - u(j));
        end
        var = w1 + w2;
    end  
end
