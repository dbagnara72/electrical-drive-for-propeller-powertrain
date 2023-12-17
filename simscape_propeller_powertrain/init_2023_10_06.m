clear all
close all
clc

beep off

%% utility
fPWM_INV = 24e3;
dead_time_INV = 0.42e-6;
dead_time = dead_time_INV;
delayINV_modA = 0;
pwm_out_lim = 1;

%% simulink model variants
use_cc_ccaller = 1;
use_cc_simulink = 1 - use_cc_ccaller;
use_obs_ccaller = 0;
use_obs_simulink = 1 - use_obs_ccaller;
use_gyro_model = 1;
use_I_F_control = 0;

%% simulink configuration
% simlength = 2;
simlength = 1;
% simlength = 0.75;

options = bodeoptions;
options.FreqUnits = 'Hz';
ts_inv = 1/fPWM_INV;
tsample = ts_inv;

if (use_gyro_model)
tc = ts_inv/40; % mandatory for stability of the solver
else
tc = ts_inv/400; % mandatory for stability of the solver
end

t_measure = simlength;
Nc = floor(t_measure/tc);
Ns_inv = floor(t_measure/ts_inv);
Nsample = floor(t_measure/tsample);
s=tf('s');
z=tf('z',ts_inv);



%% current quantization
Imax_adc = 96;
CurrentQuantization = Imax_adc/2^11;

%% DClink
CFi = 55e-6; %210e-6
Vdc_bez = 400;
Vdc = Vdc_bez*0.9;
Vdc_max = Vdc_bez*1.25;
udc_bez = Vdc_bez;

%% motor setup
run('pmsm_data/pmsm_64Nm_2291rpm_ver_viii.m');

if (use_gyro_model)
motorc_m_scale = 3/2*Vdc_bez/ubez;
% motorc_m_scale = 2/3*Vdc_bez/ubez;
else
motorc_m_scale = 2/3*Vdc_bez/ubez;
end

%% propeller
run('propeller_data/propeller_model.m');

%% load data
run('load_data/load_data.m');

%% general control control 
% torque_mode = 0;
% id_ref_pu = 0.175;
% id_ff_omega_th = 1.1;
% bemf_obsv_enable_speed = 0.1;

torque_mode = 1;
torque_ref = 64/ibez*1.1;
angle_error_deg = 8;
angle_error = angle_error_deg/180*pi;
time_init = 0.025;
id_ref_pu = 0;
id_ff_omega_th = 0.2;
bemf_obsv_enable_speed = 0.20;

%% I/F control
I_F_control = use_I_F_control;
iload_rms_ref = 0;
id_ref_if_pu = iload_rms_ref*sqrt(2)/ibez;
omega_ref_if_pu = 300*2*pi/omega_bez;

%% simulation data: speed reference, load torque 
omega_m_sim1 = omega_m_bez*0.2;
omega_m_sim2 = omega_m_bez*0.35;
omega_m_sim3 = omega_m_bez*0.5;
omega_m_sim4 = omega_m_bez*0.75;
omega_m_sim5 = omega_m_bez;


%% double integrator observer
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-500 -2000]*ts_inv);
% p2place = exp([-250 -1000]*ts_inv);
p2place = exp([-300 -1200]*ts_inv);
% p2place = exp([-125 -500]*ts_inv);
% p2place = exp([-50 -200]*ts_inv);
K = (acker(Aso',Cso',p2place))';
kg = K(1)
kw = K(2)
% kg = 0.2;
% kw = 38;

%% rotor speed control
kp_w = 1;
ki_w = 1;

%% current control
if (use_gyro_model==1)
    % kp_iq = 0.1875;
    % ki_iq = 9;
    % kp_id = 0.1875;
    % ki_id = 9;
    kp_iq = 0.15;
    ki_iq = 35;
    kp_id = 0.15;
    ki_id = 35;
else
    kp_iq = 0.15;
    ki_iq = 35;
    kp_id = 0.15;
    ki_id = 35;
    % kp_iq = 0.25;
    % ki_iq = 12;
    % kp_id = 0.25;
    % ki_id = 12;
end
CTRPIFF_CLIP_RELEASE = 0.01;

reg_cc_inv_d = ki_id/s+kp_id;
regd_cc_inv_d=c2d(reg_cc_inv_d,ts_inv);
p_cc_inv_d = ubez/(s*Ld_norm + Rs_norm);
pd_cc_inv_d = c2d(p_cc_inv_d,ts_inv);
inv_cc_plantd_d = minreal(pd_cc_inv_d*regd_cc_inv_d);
figure; margin(inv_cc_plantd_d); 
set(cstprefs.tbxprefs,'FrequencyUnits','Hz');
legend('id-cc');
grid on;


%% Field Weakening control
kp_fw = 0.5;
ki_fw = 1.8;

%% Back EMF model
emf_fb_p = 0.2;
emf_p = emf_fb_p*4/10;

emf_fb_p_ccaller_1 = 0.2;
emf_p_ccaller_1 = emf_fb_p_ccaller_1*4/10;

emf_fb_p_ccaller_2 = 0.4;
emf_p_ccaller_2 = emf_fb_p_ccaller_2*4/10;

% phase_compensation_omega = -pi/4;
phase_compensation_omega = 0;

%% omega hat filter
omega_flt_fcut = 4;
omega_flt_g0 = omega_flt_fcut * ts_inv * 2*pi;
omega_flt_g1 = 1 - omega_flt_g0;


%% Reference filter
fcut_reference_flt = 4;
g0_reference_flt = fcut_reference_flt * ts_inv * 2*pi;
g1_reference_flt = 1 - g0_reference_flt;

%% Module selection
Tambient = 1; % celsius
DThs_init = 0; % Heat sink initial delta temperature [Celsius]
run('module_data/module_CAB016M12FM3.m');
run('heatsink_data/heatsink_data.m');

%% Lithium Ion Battery
number_of_cells = 92; % nominal is 100

% stato of charge init
soc_init = 0.5; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 10; %Hr*A

Vdc_nom =Vdc_bez;
Pnom = 20e3;
Idc_nom = Pnom/Vdc_nom;
Rmax = Vdc_nom^2/(Pnom*0.1);
Rmin = Vdc_nom^2/(Pnom);

E_1 = -1.031;
E0 = 3.685;
% E0 = 3.485;
% E1 = 0.15;
E1 = 0.2156;
E2 = 0;
E3 = 0;
Elog = -0.05;
alpha = 35;

R0 = 0.15;
R1 = 0.15;
C1 = 0.1;
M = 125;


q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure; 
plot(Zmodel,ocv_model,'LineWidth',2);
xlabel('state of charge [p.u.]');
ylabel('open circuit voltage [V]');
title('open circuit voltage(state of charge)');
grid on


%% C-Caller
open_system('fb_sv_bemf_obsv_pmsm');
Simulink.importExternalCTypes('fb_sv_bemf_obsv_pmsm','Names',{'mavgflt_output_t'});
Simulink.importExternalCTypes('fb_sv_bemf_obsv_pmsm','Names',{'bemf_mb_obsv_output_t'});
Simulink.importExternalCTypes('fb_sv_bemf_obsv_pmsm','Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes('fb_sv_bemf_obsv_pmsm','Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes('fb_sv_bemf_obsv_pmsm','Names',{'global_state_machine_output_t'});





























