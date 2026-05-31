% evaluate_all.m — Run ATE evaluation on all VIRAL dataset sequences.
%
% Based on the original evaluate_all.m from ntu-aris/viral_eval.
% Adapted to read D-LIO results from Dataset/VIRAL/result_* directories.
%
% Usage: Open this file in MATLAB and press Run (F5).
%        Results are printed to the console and saved to evaluation_result.mat.
%
% See also: evaluate_one

clear all;
close all;

tic

path        = matlab.desktop.editor.getActiveFilename;
this_dir    = path(1: end - length(mfilename) - 2);
cd(this_dir);

% Select which result set to evaluate:
tests       = dir([this_dir 'Dataset/VIRAL/result_*']);           % Current results
% tests       = dir([this_dir 'Dataset/VIRAL/paper_results/result_*']); % Archived paper results
tests_count = length(tests);

ATE_POSE    = cell(tests_count, 2);

fprintf('Number of tests: %d\n', length(tests));

% delete(gcp)
% parpool(6);

for n=3:3
% parfor n=1:tests_count
    
    disp(['Evaluando archivo: ', tests(n).name]);
    P_h_ate = evaluate_one(n, tests(n));
            
    ATE_POSE(n, :) = {tests(n).name, P_h_ate};
end


save('evaluation_result.mat', 'ATE_POSE');

ATE_POSE

toc
