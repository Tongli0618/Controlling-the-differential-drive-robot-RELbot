clear;
close all;

%% Read data
% Open the file for reading
fileA = "Ass2_EVL_Stress_10000.txt";
fileB = "Ass2_EVL_10000.txt";

fidA = fopen(fileA, 'r');
fidB = fopen(fileB, 'r');

% Read the file. Specify the format of the data to match your file's structure
dataA = textscan(fidA, 'Before: %f ns, After: %f ns, Load: %f ns, Period: %f ns');
dataB = textscan(fidB, 'Before: %f ns, After: %f ns, Load: %f ns, Period: %f ns');

% Close the file
fclose(fidA);
fclose(fidB);

% Extract the columns into separate variables, if needed
beforeTimestampsA = dataA{1};
afterTimestampsA = dataA{2};
actualIntervalsA = dataA{4};
intendedIntervalA = 1000000;

beforeTimestampsB = dataB{1};
afterTimestampsB = dataB{2};
actualIntervalsB = dataB{4};
intendedIntervalB = 1000000;

%% Calculate
deviationsA = intendedIntervalA - actualIntervalsA;
deviationsB = intendedIntervalB - actualIntervalsB;

%% Plot deviations over time
figure;
hold on;
plot(actualIntervalsA(1:100));
plot(actualIntervalsB(1:100));
set(gca, 'TickLabelInterpreter', 'latex');
legend("no stress", "stress", 'Interpreter', 'latex');
title('Actual intervals', 'Interpreter', 'latex')
xlabel('Iteration', 'Interpreter', 'latex');
ylabel('Interval [ns]', 'Interpreter', 'latex');
saveas(gcf, "test.png");

%% Plot histogram of deviations
figure;
edges = linspace(-20000, 20000, 51);
histogram(deviationsA,'BinEdges',edges);
set(gca, 'TickLabelInterpreter', 'latex');
title('Distribution of deviations', 'Interpreter', 'latex')
xlabel('Deviation [ns]', 'Interpreter', 'latex');
ylabel('Occurances', 'Interpreter', 'latex');
saveas(gcf, fileA + "_histogram.png");

%%
figure;

% 为数据集A创建直方图
subplot(1, 2, 1); % 分为1行2列，这是第1个图
histogram(deviationsA, 'BinEdges', edges);
set(gca, 'TickLabelInterpreter', 'latex');
title('Posix computation loop 10,000 with Stress', 'Interpreter', 'latex')
xlabel('Deviation [ns]', 'Interpreter', 'latex');
ylabel('Occurrences', 'Interpreter', 'latex');
xlim([-20000 20000]); 
ylim([0 max([histcounts(deviationsA, edges), histcounts(deviationsB, edges)])+200]); % 保证y轴的范围一致

% 为数据集B创建直方图
subplot(1, 2, 2); 
histogram(deviationsB, 'BinEdges', edges);
set(gca, 'TickLabelInterpreter', 'latex');
title('Posix computation loop 10,000 without Stress', 'Interpreter', 'latex')
xlabel('Deviation [ns]', 'Interpreter', 'latex');
ylabel('Occurrences', 'Interpreter', 'latex');
xlim([-20000 20000]); 
ylim([0 max([histcounts(deviationsA, edges), histcounts(deviationsB, edges)])+200]); % 保证y轴的范围一致

% 保存图像
saveas(gcf, 'datasets_histogram_comparison.png');

