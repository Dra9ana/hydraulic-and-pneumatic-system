clc;clear;close all;
%% inicijalizacija parametara

Bp  = 30;     % koeficijent viskoznog prigusenja
K   = 0;      % koeficijent unutrasnje krutosti aktuatora
Ap  = 6.5e-3; % povrsina klipa
Mt  = 10;     % masa klipa sa klipnjacom
Vt  = 4e-3;   % zapremina cilindra
Kc  = 1.8e-12;% koeficijent protok-pritisak
Ctp = 2e-13;  % koeficijent totalnog curenja
Kq  = 1;      % koeficijent protok-pomeraj ventila
Kt  = 3.86e-2;% konstanta momenta motora
Kf  = 2.5e3;  % krutost povratne opruge
R   = 10;     % otpornost namotaja
r   = 5e-3;   % duzina povratne sprege
Be  = 13e8;   % koeficijent stisljivosti fluida
m   = 20;     % masa blokova
k   = 4e5;    % koeficijent elasticnosti opruga
mu  = 0.02;   % koeficijent trenja
b   = 100;    % koeficijent prigusenja
g   = 9.81;   % gravitaciono ubrzanje
L   = 0.2;    % rastojanje koje treba da predje
T   = 2;      % vreme kretanja
Tacc= 0.25*T;  % vreme ubrzavanja
Umax= 24;
p   = 160e-5;

%% matrica A
a21 = -4*k/(3*Mt);
a22 = -(Bp+b)/Mt;
a23 = Ap/Mt;
a24 = 4*k/(3*Mt);
a25 = b/Mt;

a32 = -4*Be*Ap/Vt;
a33 = -4*Be*(Ctp+Kc)/Vt;

a51 = 4*k/(3*m);
a52 = b/m;
a54 = -10*k/(3*m);
a55 = -(b+mu)/m;
a56 = 2*k/m;

a74 = 4*k/m;
a76 = -6*k/m;
a77 = -2*mu/m;

A=[ 0   1   0   0   0   0   0;
    a21 a22 a23 a24 a25 0   0;
    0   a32 a33 0   0   0   0;
    0   0   0   0   1   0   0;
    a51 a52 0   a54 a55 a56 0;
    0   0   0   0   0   0   1;
    0   0   0   a74 0   a76 a77];
    
%% matrica B
b31 = 4*Be*Kq*Kt/(R*r*Kf*Vt);
B = [0;0;b31;0;0;0;0];
%% matrica C
C = [0,0,0,1,0,0,0];

%% funkcija prenosa
s=tf('s');
G = C*(s*eye(7)-A)^(-1)*B;

figure(1)
pzmap(G);
figure(2)
step(G/(G+1));
%% trapezna formula
syms a
L1 = 1/2*a*Tacc^2;
L2 = a*Tacc*(T - 2*Tacc);
L3 = L1;
solve(L == (L1 + L2 + L3), a);
v = a*Tacc;

%% Koeficijenti PID regulatora
Kkr = 140; 
Tkr = 0.035; 
Kp = 0.6*Kkr;
Ti = Tkr/2;
I = Kp/Ti;
Td = Tkr/8;
DD = Kp*Td;
%% 
sim('glavni_blok.slx');
% stanja z1 z2 z2_dot z3 z4 z5 z5_dot z6 z7 z7_dot

figure(3)
subplot(2,2,3)
plot(tout,stanja(:,1));
title('pozicija klipa');ylim([0,0.5]);
ylabel('pozicija[m]');xlabel('vreme[s]');
grid on;
subplot(2,2,[1,2])
plot(tout,stanja(:,5));
hold on;
plot(tout,trajektorija(:,1));
title('pozicija');ylim([0,0.5]);
ylabel('pozicija[m]');xlabel('vreme[s]');
legend('prvi blok','referentna vrednost');
grid on; hold off;
subplot(2,2,4)
plot(tout,stanja(:,8));
title('pozicija drugog bloka');ylim([0,0.2]);
ylabel('pozicija[m]');xlabel('vreme[s]');
grid on; 


figure(4)
plot(tout,stanja(:,2));
hold on;
plot(tout,stanja(:,6));
hold on;
plot(tout,stanja(:,9));
hold on;
plot(tout,trajektorija(:,2));
title('brzina');ylim([0,0.3]);
ylabel('brzina[m/s]');xlabel('vreme[s]');
legend('klip','prvi blok','drugi blok','referentna vrednost');
grid on; hold off;


figure(5)
subplot(2,2,3)
plot(tout,stanja(:,3));
title('ubrzanje klipa');
ylabel('ubrzanje[m/s^2]');xlabel('vreme[s]');
grid on;
subplot(2,2,[1,2])
plot(tout,stanja(:,7));
hold on;
plot(tout,trajektorija(:,3));
title('ubrzanje');
ylabel('ubrzanje[m/s^2]');xlabel('vreme[s]');
legend('prvi blok','referentna vrednost');
grid on;

subplot(2,2,4)
plot(tout,stanja(:,10));
title('ubrzanje drugog bloka');
ylabel('ubrzanje[m/s^2]');xlabel('vreme[s]');
grid on;

figure(6)
plot(tout,stanja(:,4));
title('PL');
ylabel('pritisak[Pa]');xlabel('vreme[s]');
grid on;
figure(7)
plot(tout,stanja(:,11));
title('FL');
ylabel('sila[N]');xlabel('vreme[s]');
grid on;

Fmax=max(stanja(:,11));
Fmin=min(stanja(:,11));%0 -> sila je uvek pozitivna i desava se izvlacenje
vmax=max(stanja(:,2));
xpmax=max(stanja(:,1));
eta_cf=0.9;
eta_pv=0.95;
eta_pm=0.8;
eta_cv=1;
p=160*10^5;%Pa
n=2000/60; %obrtaja u sekundi
h=0.4;
D=sqrt(4*Fmax/(eta_cf*pi*p));
disp(D*1000);
D=70e-3;
Ap=D*D*pi/4;
V=Ap*vmax/(n*eta_pv);
Pin=p*Ap*vmax*eta_cv/eta_pm;
