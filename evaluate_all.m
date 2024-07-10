clear all;
close all;

tic

path        = matlab.desktop.editor.getActiveFilename;
this_dir    = path(1: end - length(mfilename) - 2);
cd(this_dir);

tests       = dir([this_dir 'result_*']);
tests_count = length(tests);

ATE_POSE    = cell(tests_count, 2);

fprintf('Number of tests: %d\n', length(tests));

% delete(gcp)
% parpool(6);

for n=1:tests_count
% parfor n=1:tests_count
    
    P_h_ate = evaluate_one(n, tests(n));
            
    ATE_POSE(n, :) = {tests(n).name, P_h_ate};
end


save('evaluation_result.mat', 'ATE_POSE');
for i=1:length(ATE_POSE)
ATE_POSE{i}
iter=size(ATE_POSE{i,2});
for j=1:iter(2)
ATE_POSE{i,2}{j}
end

end
toc