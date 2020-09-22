clc;
clear;
close all;

% Define side of cubical box
side = 0.050;

% Cooridinates of the vertices of the cubical box
A = [-side/2, side/2, side];
B = [side/2, side/2, side];
C = [side/2, side/2, 0];
D = [-side/2, side/2, 0];
E = [side/2, -side/2, side];
F = [side/2, -side/2, 0];
G = [-side/2, -side/2, 0];
H = [-side/2, -side/2, side];
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
save("box_vrtx_nrml.mat", "blk_vrtx", "blk_nrm");
