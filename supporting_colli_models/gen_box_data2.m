clc;
clear;
close all;

% Define side of cubical box
% height = 0.200; width = 0.800; thick = 0.020;
% height = 0.200; width = 0.310; thick = 0.020;
% height = 0.050; width = 0.210; thick = 0.050;
% height = 0.010; width = 0.180; thick = 0.010;
% height = 0.050; width = 0.180; thick = 0.050;

% Generate data for experiment
% height = 0.015; width = 0.290; thick = 0.015;  % grasped box
% height = 0.400; width = 1.600; thick = 0.020;  % top and bottom walls
height = 0.215; width = 0.6660; thick = 0.020;   % side walls


% Cooridinates of the vertices of the cubical box
A = [-thick/2, width/2, height];
B = [thick/2, width/2, height];
C = [thick/2, width/2, 0];
D = [-thick/2, width/2, 0];
E = [thick/2, -width/2, height];
F = [thick/2, -width/2, 0];
G = [-thick/2, -width/2, 0];
H = [-thick/2, -width/2, height];
blk_vrtx = [A; B; C; D; E; F; G; H];

% Define the normals to the surfaces
EBCF_nrml = [1, 0, 0];
BCDA_nrml = [0, 1, 0];
EBAH_nrml = [0, 0, 1];
ADGH_nrml = [-1, 0, 0];
EFGH_nrml = [0, -1, 0];
FCDG_nrml = [0, 0, -1];
blk_nrm = [EBCF_nrml; BCDA_nrml; EBAH_nrml; ADGH_nrml; EFGH_nrml; FCDG_nrml];

% Visualize
box3d(blk_vrtx', 2);

% Save the data in a mat file
% save("box_vrtx_nrml4.mat", "blk_vrtx", "blk_nrm");
% save("box_vrtx_nrml5.mat", "blk_vrtx", "blk_nrm");
% save("box_vrtx_nrml6.mat", "blk_vrtx", "blk_nrm");
% save("box_grasped2.mat", "blk_vrtx", "blk_nrm");
% save("box_grasped3.mat", "blk_vrtx", "blk_nrm");
% save("box_grasped29.mat", "blk_vrtx", "blk_nrm");

% Saved daata for experiment
% save("box_grasped3_exp.mat", "blk_vrtx", "blk_nrm"); % grasped box
% save("box_vrtx_nrml3_exp.mat", "blk_vrtx", "blk_nrm"); % top and bottom walls
save("box_vrtx_nrml6_exp.mat", "blk_vrtx", "blk_nrm"); % side walls
