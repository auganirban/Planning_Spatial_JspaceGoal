clc;
clear;


% % % % Generate Obstacle 1
% % % table_top_from_base = -0.2;
% % % obstable_ht_from_base = 0.6;
% % % obstable_ht_from_table_top = obstable_ht_from_base - table_top_from_base;
% % % blk_vrtx = [-0.300, 0.200, obstable_ht_from_table_top;
% % %     0.300, 0.200, obstable_ht_from_table_top;
% % %     0.300, 0.200, 0;
% % %    -0.300, 0.200, 0;
% % %     0.300,-0.200, obstable_ht_from_table_top;
% % %     0.300,-0.200, 0;
% % %    -0.300,-0.200, 0;
% % %    -0.300,-0.200, obstable_ht_from_table_top];
% % % 
% % % blk_nrm = [1, 0, 0;
% % %     0, 1, 0;
% % %     0, 0, 1;
% % %    -1, 0, 0;
% % %     0 -1, 0;
% % %     0, 0, -1];
% % % 
% % % save("obs_nrm_vrtx3.mat","blk_vrtx","blk_nrm");

% % % % Generate Obstacle 2
table_top_from_base = -0.2;
obstable_ht_from_base = 0.6;
obstable_ht_from_table_top = obstable_ht_from_base - table_top_from_base;
blk_vrtx = [-0.100, 0.100, obstable_ht_from_table_top;
    0.100, 0.100, obstable_ht_from_table_top;
    0.100, 0.100, 0;
   -0.100, 0.100, 0;
    0.100,-0.100, obstable_ht_from_table_top;
    0.100,-0.100, 0;
   -0.100,-0.100, 0;
   -0.100,-0.100, obstable_ht_from_table_top];

blk_nrm = [1, 0, 0;
    0, 1, 0;
    0, 0, 1;
   -1, 0, 0;
    0 -1, 0;
    0, 0, -1];

save("obs_nrm_vrtx4.mat","blk_vrtx","blk_nrm");
