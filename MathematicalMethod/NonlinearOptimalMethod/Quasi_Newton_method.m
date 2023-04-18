% ------------------------------------------------
% Date 2019 11/08 
% Author Keigo Miyama 
% Rosenbrock関数を対象とした準ニュートン法による最適問題
% ------------------------------------------------
function [] = Quasi_Newton_method()
clear;
close all;
clc;

max = 2.5;
min = -2.5;

[x1range,x2range,yrange] = Create(max,min);
param = Init(max,min);
Steepest(param,x1range,x2range,yrange);

function Steepest(param,x1range,x2range,yrange)
%最急降下法による最適化
iterationMax=400;%イタレーションの最大回数
time = 0;
result.param=[];%パラメータの履歴
result.time = [];
result.f = [];
maxalpha = 10; minalpha = -10;
H = eye(2);
tic;
for i = 1:iterationMax
    time = time + i;
    J = jacob(param(1),param(2));
    if norm(J) <= 0.01
        disp('Finish');
        break;
    else
        delta = - inv(H)*J';
        alpha = Golden(maxalpha,minalpha);
        param_1(1:2) = param(1:2) + alpha*delta';
        s = param_1(1:2)' - param(1:2)';
        J_1 = jacob(param_1(1),param_1(2));
        y = J_1' - J';
        H = H + (y*y')/(y'*s) - (H*s*s'*H)/(s'*H*s); % Broyden-Fletcher-Goldfarb-Shanno-method
        %H = (y*y')/(y'*s) + (eye(2) - ((y*y')/(y'*s)))*H*(eye(2) - ((s*y')/(s'*y)));
        param(1:2) = param_1(1:2);
        f = f(param(1),param(2)) % 関数fの値を常にコマンドウィンドウで表示させている
    end
    result.param=[result.param;param];
    result.time = [result.time;time];
    result.f = [result.f; f];
    
    figure(1)
    hold off;
    contour(x1range,x2range,yrange,100); hold on;
    plot3(result.param(:,1),result.param(:,2),result.param(:,3),'.-k','markersize',10);
    xlabel('x1');ylabel('x2');
    drawnow;
    pause(0.5);
    
    figure(2)
    plot(result.time,result.f,'r--');
    xlabel('Iteration'); ylabel('f');
    grid on
end
toc

function alpha = Golden(maxalpha,minalpha)
r = 0.618; %黄金分割比
alpha1 = minalpha + (1 - r)*(maxalpha - minalpha);
alpha2 = minalpha + r*(maxalpha - minalpha);
f1 = f_alpha(alpha1);
f2 = f_alpha(alpha2);

while 1
    if f1 < f2
        maxalpha = alpha2;
        alpha2 = alpha1;
        alpha1 = minalpha + (1 - r)*(maxalpha - minalpha);
    else
        minalpha = alpha1;
        alpha1 = alpha2;
        alpha2 = minalpha + r*(maxalpha - minalpha);
    end

    if abs(maxalpha - minalpha) <= 0.01
        alpha = (maxalpha - minalpha)/2;
        break;
    end
end
    
function [x1range,x2range,yrange] = Create(max,min)
[x1range, x2range] = meshgrid(min:0.3:max);
yrange = f(x1range,x2range);

function y = f(x1,x2)
y = 100*(x2 - x1^2)^2 + (1 - x1)^2;

function y = f_alpha(x)
y = 100*(x - x^2)^2 + (1 - x)^2;

function param = Init(max,min)
% x1 = max - (max - min).*rand(1,1);
% x2 = max - (max - min).*rand(1,1);
% y=f(x1,x2);
x1 = -1.2;
x2 = 1.0;
y = f(x1,x2);

param = [x1 x2 y];

function J = jacob(x,y)
dx = -400*x*y + 400*x^3 + 2*x -2;
dy = 200*y -200*x^2;
J=[dx dy];

function H = hessian(x,y)
H = [-400*y+1200*x^2+2 -400*x;
     -400*x 200];
