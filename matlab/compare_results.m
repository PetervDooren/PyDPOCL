% Load the results from test_results.csv into a table
data1 = readtable('test_results_classic.csv');
data2 = readtable('test_results_robot_first.csv');

% Extract rows where the status field is 'no_plan_found'
noPlanFoundRows1 = data1(strcmp(data1.status, 'no_plan_found'), :);
noPlanFoundRows2 = data2(strcmp(data2.status, 'no_plan_found'), :);

% Extract rows where the status field is 'success'
SuccessRows1 = data1(strcmp(data1.status, 'success'), :);
SuccessRows2 = data2(strcmp(data2.status, 'success'), :);

nr_goals_array = 1:max(SuccessRows1.nr_goals);
for i=nr_goals_array
    subdata1 = SuccessRows1(SuccessRows1.nr_goals == i, :);
    nr_entries1(i) = height(subdata1);
    median_per_goal1(i) = median(subdata1.planning_time);
    mean_per_goal1(i) = mean(subdata1.planning_time);

    subdata2 = SuccessRows2(SuccessRows2.nr_goals == i, :);
    nr_entries2(i) = height(subdata2);
    median_per_goal2(i) = median(subdata2.planning_time);
    mean_per_goal2(i) = mean(subdata2.planning_time);
end



figure(1)
clf
hold on
plot(SuccessRows1.nr_goals, SuccessRows1.planning_time, 'bo')
plot(SuccessRows2.nr_goals, SuccessRows2.planning_time, 'ro')
plot(nr_goals_array, median_per_goal1, 'b')
plot(nr_goals_array, median_per_goal2, 'r')
plot(nr_goals_array, mean_per_goal1, 'b--')
plot(nr_goals_array, mean_per_goal2, 'r--')
xlabel("number of goals")
ylabel("planning time [s]")
yscale("log")
grid on
legend('classic', 'robot first')
