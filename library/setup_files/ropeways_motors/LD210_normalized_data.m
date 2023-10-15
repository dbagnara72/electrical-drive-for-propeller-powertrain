n_modules = 4;
Pn=418e3/n_modules;
Vn=415;
In=790/n_modules;
En = 200; 
cos_phi=0.8;
eta = 0.92;
cp=40;
wn_rpm=19.5;

omega_base = wn_rpm/60*cp*2*pi;
Z_base = Vn/sqrt(3)/In;
R = Pn*(1-eta)/3/(In^2)
L = Z_base * sin(acos(cos_phi)) / omega_base


Rs=R;
% Ld=(L + LFi_d);
% Lq=(L + LFi_d);
% Ls=(L + LFi_d);
Ld=(L);
Lq=(L);
Ls=(L);

Omega_bez=wn_rpm/60*cp*2*pi;
Omega_s_bez=Omega_bez;
Omega_r_bez=wn_rpm/60*2*pi;

omega_scale = 1;
load_angle= -14;
Ibez=In*sqrt(2);
Ibez_d=In*sqrt(2)* sind(load_angle);
Ibez_q=In*sqrt(2)* cosd(load_angle);
% Ubez=sqrt(2)*sqrt(En^2+(Lq*In*we)^2)
% Ubez=sqrt(2)*sqrt((En+Ld*Ibez_d/sqrt(2)*we)^2+(Lq*Ibez_q/sqrt(2)*we)^2)
Ubez=En*sqrt(2);
phi_bez = Ubez/Omega_bez;

Xbez=Ubez/Ibez;
Lbez=Xbez/Omega_bez;
Rs_norm = Rs/Xbez;
Ld_norm = Ld/Lbez;
Lq_norm = Lq/Lbez;
Ls_norm = Ls/Lbez;
Tnom = 205e3/n_modules;
J=2957;
tau_i_m = 1/(sqrt(J/Tnom)*Omega_r_bez)
% tau_i_m = 0

phi = En*sqrt(2)/Omega_bez;
phi_norm = phi/phi_bez

emf_fb_p = 0.2;
emf_p = 0.08;
k = 5;
kalman_gamma = 0.8;
kalman_w = kalman_gamma/k;
% kalman_w = kalman_gamma*k;
kalman_iq_est = 0.5;
% kalman_iq_est = 0;

iq_ref_norm_lim = 1;

Tload = Tnom;
emf_valid =0.1;

Vdc_nom = 650;
motorc_u_scale = Vdc_nom/sqrt(3)/Ubez
























