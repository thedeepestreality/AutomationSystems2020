time = InOutData.time;
data = InOutData.signals.values;
y = data(:,1);
u = data(:,2);

close all;
figure(1);
% stem(time, u, 'r', 'LineWidth',2);
plot(time, u, 'r', 'LineWidth',2);
grid on;
hold on;
% stem(time, y, 'k', 'LineWidth',2);
plot(time, y, 'k', 'LineWidth',2);
legend({'input','output'});