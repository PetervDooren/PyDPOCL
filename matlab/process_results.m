% Load the results from test_results.csv into a table
data = readtable('test_results.csv');

% Extract rows where the status field is 'no_plan_found'
noPlanFoundRows = data(strcmp(data.status, 'no_plan_found'), :);
% Extract rows where the status field is 'faulty_plan'
FaultyRows = data(strcmp(data.status, 'faulty_plan'), :);


% Extract rows where the status field is 'success'
SuccessRows = data(strcmp(data.status, 'success'), :);

% Extract rows that took a long planning time'
LongPlanningTimeRows = data(SuccessRows.planning_time > 10, :);

nr_goals_array = 1:max(SuccessRows.nr_goals);
for i=nr_goals_array
    subdata = SuccessRows(SuccessRows.nr_goals == i, :);
    nr_entries(i) = height(subdata);
    median_per_goal(i) = median(subdata.planning_time);
    mean_per_goal(i) = mean(subdata.planning_time);
end

figure(1)
hold on
plot(SuccessRows.nr_goals, SuccessRows.planning_time, 'bo')
plot(nr_goals_array, median_per_goal, 'b')
plot(nr_goals_array, mean_per_goal, 'b--')
xlabel("number of goals")
ylabel("planning time [s]")
