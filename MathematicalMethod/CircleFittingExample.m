function [] = CircleFittingExample()
    clear;
    close all;
    clc;

    %推定する円情報
    cx=4;
    cy=10;
    r=30;

    %円の点群の擬似情報
    x=[-10:10];
    y=cy+sqrt(r^2-(x-cx).^2);
    y=[y cy-sqrt(r^2-(x-cx).^2)];
    x=[x x];
    plot(x,y,'ro','LineWidth', 2);hold on;

    %円フィッティング
    [cxe,cye,re]=CircleFitting(x,y);

    theta=[0:0.1:2*pi 0];
    xe=re*cos(theta)+cxe;
    ye=re*sin(theta)+cye;

    plot(xe,ye,'-b');hold on;

    grid on;
    axis equal;
end

function [ cx, cy, r ] = CircleFitting(x,y)
%CIRCLEFITTING 最小二乗法による円フィッテングをする関数
% input: x,y 円フィッティングする点群
% output cx 中心x座標
%        cy 中心y座標
%        r  半径
% 参考
% 一般式による最小二乗法（円の最小二乗法）　画像処理ソリューション
% http://imagingsolution.blog107.fc2.com/blog-entry-16.html

sumx=sum(x);
sumy=sum(y);
sumx2=sum(x.^2);
sumy2=sum(y.^2);
sumxy=sum(x.*y);

F=[sumx2 sumxy sumx;
   sumxy sumy2 sumy;
   sumx  sumy  length(x)];

G=[-sum(x.^3+x.*y.^2);
   -sum(x.^2.*y+y.^3);
   -sum(x.^2+y.^2)];

T=F\G;

cx=T(1)/-2;
cy=T(2)/-2;
r=sqrt(cx^2+cy^2-T(3));

end
