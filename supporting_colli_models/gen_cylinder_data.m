clc;
clear;
close all;

% Define radius and length of the cylinder
rc = 0.020; hc = 0.050;

% Use matlab's cylinder function to generate
% points of a cylinder.
[X, Y, Z] = cylinder(rc);
Z = hc * Z;
save("cylinder_data.mat", "X", "Y", "Z");
