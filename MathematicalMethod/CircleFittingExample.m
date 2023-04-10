function [] = CircleFittingExample()
    clear;
    close all;
    clc;

    %���肷��~���
    cx=4;
    cy=10;
    r=30;

    %�~�̓_�Q�̋[�����
    x=[-10:10];
    y=cy+sqrt(r^2-(x-cx).^2);
    y=[y cy-sqrt(r^2-(x-cx).^2)];
    x=[x x];
    plot(x,y,'ro','LineWidth', 2);hold on;

    %�~�t�B�b�e�B���O
    [cxe,cye,re]=CircleFitting(x,y);

    theta=[0:0.1:2*pi 0];
    xe=re*cos(theta)+cxe;
    ye=re*sin(theta)+cye;

    plot(xe,ye,'-b');hold on;

    grid on;
    axis equal;
end

function [ cx, cy, r ] = CircleFitting(x,y)
%CIRCLEFITTING �ŏ����@�ɂ��~�t�B�b�e���O������֐�
% input: x,y �~�t�B�b�e�B���O����_�Q
% output cx ���Sx���W
%        cy ���Sy���W
%        r  ���a
% �Q�l
% ��ʎ��ɂ��ŏ����@�i�~�̍ŏ����@�j�@�摜�����\�����[�V����
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
