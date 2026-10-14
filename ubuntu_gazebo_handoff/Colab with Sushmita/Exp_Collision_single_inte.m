function d_dt = Exp_Collision_single_inte(Hbar_Nset,Hbar_Mset,psi_star,Kc,Ks,r,psi_crit,delta,pout)
p=pout(1:end);
e_N=Hbar_Nset*p;
e_M=Hbar_Mset*p;
m=10;
mhat=2;

R_psi_N=[];R_psi_M=[]; xi=[];xi_beta=[]; E=[];
for i=1:2:m
    dij=norm(e_N(i:i+1));
    psi_ij=2*asind(r/dij);
    gij=e_N(i:i+1)/dij;
    Rot_R=[cosd(psi_ij/2) -sind(psi_ij/2);sind(psi_ij/2) cosd(psi_ij/2)];
    Rot_L=[cosd(psi_ij/2) sind(psi_ij/2);-sind(psi_ij/2) cosd(psi_ij/2)];
    g_iRj=Rot_R*gij; g_iLj=Rot_L*gij;
    Pg_iRj=eye(2,2)-(g_iRj*g_iRj');
    Pg_iLj=eye(2,2)-(g_iLj*g_iLj');
    f_gij=(Pg_iLj*g_iRj)+(Pg_iRj*g_iLj);
    f_psi_ij=0.5*sind(2*psi_ij)/(1+cosd(psi_ij));
    R_psi_N=blkdiag(R_psi_N,f_psi_ij*f_gij);
    c=(i+1)/2;
    E_ij=(cosd(psi_ij))^2-(cosd(psi_star(c)))^2;
    if E_ij>0
        q=1;
    else
        q=0;
    end
    K=(q*Kc(c))+((1-q)*Ks(c));
    xi_temp=4*E_ij/(K^2-E_ij^2);
    xi=[xi;xi_temp];
    E=[E;E_ij];
end

for i=1:2:mhat
    dik=norm(e_M(i:i+1));
    psi_ik=2*asind(r/dik);
    gik=e_M(i:i+1)/dik;
    Rot_R=[cosd(psi_ik/2) -sind(psi_ik/2);sind(psi_ik/2) cosd(psi_ik/2)];
    Rot_L=[cosd(psi_ik/2) sind(psi_ik/2);-sind(psi_ik/2) cosd(psi_ik/2)];
    g_iRk=Rot_R*gik; g_iLk=Rot_L*gik;
    Pg_iRk=eye(2,2)-(g_iRk*g_iRk');
    Pg_iLk=eye(2,2)-(g_iLk*g_iLk');
    f_gik=(Pg_iLk*g_iRk)+(Pg_iRk*g_iLk);
    f_psi_ik=0.5*sind(2*psi_ik)/(1+cosd(psi_ik));
    R_psi_M=blkdiag(R_psi_M,f_psi_ik*f_gik);
    if psi_ik>psi_crit
        beta_ik=(cosd(psi_ik))^2-(cosd(psi_crit))^2;
    else
        beta_ik=0;
    end
    xi_beta_temp=4*beta_ik/(delta^2-beta_ik^2);
    xi_beta=[xi_beta;xi_beta_temp];
end

%d_dt=-100*(Hbar_Nset'*R_psi_N*E);                             % Plain formation control
%d_dt=-(Hbar_Nset'*R_psi_N*xi);                            % Connectivity presevation
d_dt=-(Hbar_Nset'*R_psi_N*xi+Hbar_Mset'*R_psi_M*xi_beta);  % Connectivity+Collision avoidance

end