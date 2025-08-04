% Load the results from test_results.csv into a table
data = readtable('test_results.csv');


% Extract rows where the status field is 'no_plan_found'
noPlanFoundRows = data(strcmp(data.status, 'no_plan_found'), :);
% Extract rows where the status field is 'faulty_plan'
FaultyRows = data(strcmp(data.status, 'faulty_plan'), :);


% Extract rows where the status field is 'faulty_plan'
SuccessRows = data(strcmp(data.status, 'success'), :);

figure(1)
plot(SuccessRows.nr_goals, SuccessRows.planning_time, 'o')
xlabel("number of goals")
ylabel("planning time [s]")
