truth_d1 = load('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/data/truth/drive_1.xyz');
truth_d2 = load('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/data/truth/drive_2.xyz');
truth_d3 = load('/home/rmw/Documents/git/Enabling-Robust-State-Estimation-through-Measurement-Error-Covariance-Adaptation/data/truth/drive_3.xyz');

nom = [856514.1467,-4843013.0689, 4047939.8237];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% GET ICE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ice_d1_lq = load('/home/rmw/Downloads/1_lq_ice.xyz');
[~, ice_d1lq_rh, ice_d1lq_rt] = getError(truth_d1, ice_d1_lq, truth_d1(1,2:end));


ice_d2_lq = load('/home/rmw/Downloads/2_lq_ice.xyz');
[~, ice_d2lq_rh, ice_d2lq_rt] = getError(truth_d2, ice_d2_lq, truth_d2(1,2:end));


ice_d3_lq = load('/home/rmw/Downloads/3_lq_ice.xyz');
[~, ice_d3lq_rh, ice_d3lq_rt] = getError(truth_d3, ice_d3_lq, truth_d3(1,2:end));


% ICE Hor
dataset = {'D1_LQ';'D2_LQ';'D3_LQ'};
med_ = [median(ice_d1lq_rh);median(ice_d2lq_rh); median(ice_d3lq_rh)];
mean_ = [mean(ice_d1lq_rh); mean(ice_d2lq_rh); mean(ice_d3lq_rh)];
var_ = [var(ice_d1lq_rh); var(ice_d2lq_rh); var(ice_d3lq_rh)];
max_ = [max(ice_d1lq_rh); max(ice_d2lq_rh); max(ice_d3lq_rh)];

Inc_Covariance_Results = table(dataset, med_, mean_, var_, max_)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%% GET L2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
l2_d1_lq = load('/home/rmw/Downloads/1_lq_l2.xyz');
[~, l2_d1lq_rh, l2_d1lq_rt] = getError(truth_d1, l2_d1_lq, truth_d1(1,2:end));


l2_d2_lq = load('/home/rmw/Downloads/2_lq_l2.xyz');
[~, l2_d2lq_rh, l2_d2lq_rt] = getError(truth_d2, l2_d2_lq, truth_d2(1,2:end));


l2_d3_lq = load('/home/rmw/Downloads/3_lq_l2.xyz');
[~, l2_d3lq_rh, l2_d3lq_rt] = getError(truth_d3, l2_d3_lq, truth_d3(1,2:end));


% ICE Hor
dataset = {'D1_LQ';'D2_LQ';'D3_LQ'};
med_ = [median(l2_d1lq_rh);median(l2_d2lq_rh); median(l2_d3lq_rh)];
mean_ = [mean(l2_d1lq_rh); mean(l2_d2lq_rh); mean(l2_d3lq_rh)];
var_ = [var(l2_d1lq_rh); var(l2_d2lq_rh); var(l2_d3lq_rh)];
max_ = [max(l2_d1lq_rh); max(l2_d2lq_rh); max(l2_d3lq_rh)];

L2_Covariance_Results = table(dataset, med_, mean_, var_, max_)
