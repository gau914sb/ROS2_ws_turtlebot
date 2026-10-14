%pstar=[1;1;  8;1;  12;12;  1;8];
%pstar=[28;35;  34;35;  37;25;  25;25];
pstar=[0.4;0.3;  -0.4;0.3;  -0.7;-0.4;  0.7;-0.4];
H_Nset=[-1 1 0 0;0 -1 1 0;0 0 -1 1;1 0 0 -1;0 1 0 -1];
Hbar_Nset=kron(H_Nset,eye(2,2));

H_Mset=[-1 0 1 0];
Hbar_Mset=kron(H_Mset,eye(2,2));

%p=[1;0;  9.5;0.5;  9;7;  0.5;7.5];
%p=[2;2;  9;2;  9;9;  2;9];
%p=[1;2;  8.5;1.5;  10;8;  2;10];
%p=[1;2;  7.5;1.5;  10;12;  2;10];
%p=[0;0;  9.5;0;  9.5;9.5;  0;9.5];
%p=[1;2;  6;0;  10;6;  5;10];
%p=[5;5;  18.5;5;  18.5;8.5;  5;8.5];
%p=[6.6;2.2;  4.4;4.4;  2.2;2.2;  4.4;0];
%p=[12;2.4;  2.2;2.2;  7.5;1.9;  4.4;0];
%p=[15;15;  25;9;  19;3.5;  8;7];
% p=[0.65;0.2;  -0.65;0.2;  -0.65;-0.2; 0.65;-0.2];
%p=[0.7;0.2;  -0.6;0.4;  0.1;0.1;  -0.45;-0.4];
%p=0.2*[4.5;0.5;  -5.3;0.3;  0;0;  -3.1;-1.9];
p=[-0.3;0;  -0.6;0.4;  0.1;-0.2;  -0.4;-0.4];

% ================= CERTIFIED initial conditions (Rcrit = 0.4) =================
% Certified by simulation to t=4000: reach pstar (err ~1e-6), keep every Nset
% edge inside [Rs,Rc] for ALL t, and keep d_13 >= Rs+r = 0.295 for ALL t.
%p=[-0.7328;-0.8677;  -0.3726;0.4088;  -0.0123;0.6353;  -0.5090;-0.3327];
%p=[0.4666;-0.8363;  -0.2531;-0.6065;  0.8978;-0.6408;  -0.4602;-0.2570];
%p=[-0.8304;-0.6926;  0.0995;0.2466;  -0.3154;0.2582;  -0.2663;-0.6648];
%p=[-0.3772;0.5378;  -0.1528;0.0958;  0.3120;0.0331;  -0.4363;0.8629];
%p=[-0.2317;-0.7363;  0.2148;-0.0812;  -0.1289;0.6337;  -0.6520;0.2105];
%p=[-0.8037;-0.5050;  -0.2053;0.4307;  0.1978;-0.8474;  -0.8190;-0.0863];
%p=[0.3972;-0.3150;  0.3449;0.0039;  -0.0918;0.8417;  -0.6010;-0.0231];
%p=[-0.0603;0.3921;  -0.1151;-0.2446;  0.2853;-0.6069;  -0.8796;0.1664];
%p=[0.1405;-0.8961;  -0.2073;-0.3290;  -0.2027;-0.8086;  0.0976;0.2235];
%p=[0.4490;0.8234;  -0.2615;0.2813;  -0.1043;-0.5129;  0.7007;0.4426];
%
% ---- AVOID: legal at t=0 but stall for EVERY Rcrit in (0.295, 1.304) ----
%p=[-0.160;-0.469;  -0.831;0.677;  -0.058;0.086;  -0.320;0.452];   % err 1.12
%p=[-0.577;-0.277;  0.807;0.132;  -0.288;-0.411;  0.814;-0.100];   % err 1.28
%p=[0.865;0.028;  0.038;0.714;  0.437;0.145;  -0.132;0.681];       % err 1.28
%
% ---- AVOID: legal at t=0 but LOSE connectivity (d_ij grows past Rc=1.6) ----
%p=[-0.879;-0.554;  0.346;-0.539;  -0.235;-0.893;  0.594;-0.622];
%p=[-0.107;-0.469;  -0.176;-0.726;  0.842;-0.513;  0.309;-0.359];
%p=[0.673;0.292;  -0.663;0.621;  0.801;0.727;  0.125;-0.638];
%p=[-0.554;0.770;  0.094;-0.575;  0.691;0.255;  0.125;-0.223];
% =============================================================================
% NOTE: the current p above is TRAPPED for Rcrit in [0.325,0.435], which
% includes the default Rcrit=0.4 set below. Use Rcrit=0.5 for this p.

d_0=[];
e_0=Hbar_Nset*p;
for i=1:2:10
    d0_temp=norm(e_0(i:i+1));
    d_0=[d_0;d0_temp];
end
d13_0=norm(p(1:2)-p(5:6));


estar=Hbar_Nset*pstar;

%r=1; Rs=2; Rc=15; Rcrit=7.5;
r=0.135; Rs=0.16; Rc=1.6; Rcrit=0.4;
Rc_bar=1-(2*(r/(Rc-r))^2);
Rs_bar=1-(2*(r/(Rs+r))^2);
Rcrit_bar=1-(2*(r/Rcrit)^2);
psi_min=acosd(Rc_bar); psi_max=acosd(Rs_bar); psi_crit=acosd(Rcrit_bar);
psi_star=[]; dstar=[];
Kc=[]; Ks=[];
for i=1:2:10
    dstar_temp=norm(estar(i:i+1));
    temp1=2*asind(r/norm(estar(i:i+1)));
    psi_star=[psi_star;temp1];
    kc=Rc_bar^2-(cosd(temp1))^2;
    Kc=[Kc;kc];
    ks=(cosd(temp1))^2-Rs_bar^2;
    Ks=[Ks;ks];
    dstar=[dstar;dstar_temp];
end
delta=Rs_bar^2-Rcrit_bar^2;

tspan=[0 2000];

[t, pout]=ode45(@(t,pout) Exp_Collision_single_inte(Hbar_Nset,Hbar_Mset,psi_star,Kc,Ks,r,psi_crit,delta,pout),tspan,p);
%[t, pout]=ode45(@(t,pout) Collision_double_inte(Hbar_Nset,Hbar_Mset,psi_star,Kc,Ks,r,psi_crit,delta,pout,n),tspan,[p;v]);

d_end=[];
p_end=pout(end,:);
e_end=Hbar_Nset*p_end';
for i=1:2:10
    d_temp=norm(e_end(i:i+1));
    d_end=[d_end;d_temp];
end

x1=[p(1) p(1) p(3) p(3) p(5);p(3) p(7) p(5) p(7) p(7)];
y1=[p(2) p(2) p(4) p(4) p(6);p(4) p(8) p(6) p(8) p(8)];

x=[pout(end,1) pout(end,1) pout(end,3) pout(end,3) pout(end,5);pout(end,3) pout(end,7) pout(end,5) pout(end,7) pout(end,7)];
y=[pout(end,2) pout(end,2) pout(end,4) pout(end,4) pout(end,6);pout(end,4) pout(end,8) pout(end,6) pout(end,8) pout(end,8)];

plot(x1,y1,'k--','LineWidth',0.75,'HandleVisibility','off')
% legend('off','off','off','off','off')
hold on

plot(x,y,'k','LineWidth',1,'HandleVisibility','off')
% legend('off','off','off','off','off')
hold on

plot(p(1,1),p(2,1),'bo','HandleVisibility','off')
legend('off')
hold on

plot(p(3,1),p(4,1),'go','HandleVisibility','off')
legend('off')
hold on

plot(p(5,1),p(6,1),'ro','HandleVisibility','off')
legend('off')
hold on

plot(p(7,1),p(8,1),'co','HandleVisibility','off')
legend('off')
hold on

plot(pout(:,1),pout(:,2),'b','HandleVisibility','off')
legend('off')
hold on

plot(pout(end,1),pout(end,2),'bo','markerfacecolor','blue')
legend('off')
hold on

plot(pout(:,3),pout(:,4),'g','HandleVisibility','off')
legend('off')
hold on

plot(pout(end,3),pout(end,4),'go','markerfacecolor','green')
legend('off')
hold on

plot(pout(:,5),pout(:,6),'r','HandleVisibility','off')
legend('off')
hold on

plot(pout(end,5),pout(end,6),'ro','markerfacecolor','red')
legend('off')
hold on

plot(pout(:,7),pout(:,8),'c','HandleVisibility','off')
legend('off')
hold on

plot(pout(end,7),pout(end,8),'co','markerfacecolor','cyan')
legend('off')
axis equal