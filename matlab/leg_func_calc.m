% 本程序用于腿部结构解算，用电机所能获取的phi1,phi4,dphi1,dphi4
% 计算腿长l0和腿部角度phi0(leg_pos)，及其移动速度dl0和dphi0(leg_spd)
% 并求得VMC所需函数leg_conv，可由推力F和扭矩Tp转换为电机扭矩T

clear;

% 设定所需符号
syms phi1 phi2 phi3 phi4;
syms dphi1 dphi4;
l1=0.05;l2=0.105;l3=l2;l4=l1;l5=0.06; % 腿部杆长
syms xc yc xb yb xd yd; % c,b,d三点的xy坐标

%进行几何计算
xb=l1*cos(phi1);
yb=l1*sin(phi1);
xd=l5+l4*cos(phi4);
yd=l4*sin(phi4);

A0=2*l2*(xd-xb);
B0=2*l2*(yd-yb);
C0=l2^2+(xd-xb)^2+(yd-yb)^2-l3^2;
phi2=2*atan((B0+sqrt(A0^2+B0^2-C0^2))/(A0+C0));

xc=xb+l2*cos(phi2);
yc=yb+l2*sin(phi2);

l0=sqrt((xc-l5/2)^2+yc^2);
phi0=atan2(yc,(xc-l5/2));

% 求得腿部姿态 [l0; phi0] = leg_pos(phi1, phi4)
pos=[l0; phi0];
matlabFunction(pos,'File','leg_pos');

% 计算雅可比矩阵
J11=diff(l0,phi1);
J12=diff(l0,phi4);
J21=diff(phi0,phi1);
J22=diff(phi0,phi4);
J=[J11 J12; J21 J22];

% 求得腿部运动速度 [dl0; dphi0] = leg_spd(dphi1, dphi4, phi1, phi4)
spd=J*[dphi1; dphi4];
matlabFunction(spd,'File','leg_spd');

% 求得VMC转换矩阵 [T1; T2] = leg_conv(F, Tp, phi1, phi4)
syms F Tp;
T=J'*[F; Tp];
matlabFunction(T,'File','leg_conv');