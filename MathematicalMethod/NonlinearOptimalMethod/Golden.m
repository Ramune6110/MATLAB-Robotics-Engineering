function [] = Golden()
close all;
clear al;

a = -50;
b = 100;

[minX,minY] = Goldendate(a,b);

x = [a:0.1:b];
y = [];

for i = 1:length(x)
    y = [y f(x(i))];
end

plot(x,y,'-r', 'markersize', 15);hold on;
plot(minX,minY,'.b', 'markersize', 15);hold on;
grid on;

function [minX,minY] = Goldendate(a,b)

%内分点の計算
%黄金率
GOLDEN_RATIO = 1.6180339887498948482045868343656;
x1 = (a-b)/(GOLDEN_RATIO + 1.0) + b;
x2 = (a-b)/GOLDEN_RATIO + b;

f1 = f(x1);
f2 = f(x2);

while 1
    %ループを回して両点を更新
    if f1 < f2
        a = x2;
        x2 = x1;
        f2 = f1;
        x1 = (a - b)/(GOLDEN_RATIO + 1.0) + b;
        f1 = f(x1);
    else
        b = x1;
        x1 = x2;
        f1 = f2;
        x2 = (a - b)/GOLDEN_RATIO + b;
        f2 = f(x2);
    end
    %収束判定
    if abs(a-b)<=10^-3
        minX=(a+b)/2;
        minY=f((a+b)/2);
        break
    end
end

function f = f(x)
%目的関数
f=(x-2)^2+5;


