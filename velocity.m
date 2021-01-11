close all; clc;

data = csvread("velocity-newsesst.csv");

data = data(200:1200, :);
t = data(:, 1);
t = t - t(1);
dt = diff(t);
pos = data(:, 2);
vel = data(:, 3);


vel_matlab = diff(pos) ./ diff(t);
vel_matlab(end+1) = 0;

plot(t, pos,  'LineWidth', 2); hold on;
plot(t, vel, 'LineWidth', 2);

prev_v = 0;
prev_t = t(1);
prev_pos = pos(1);
alpha = 0.1;

vel_out = zeros(size(vel, 1), 1);
for i=2:1:size(pos,1)
    curr_t = t(i);
    curr_pos = pos(i);
    curr_v = (curr_pos - prev_pos) / (curr_t - prev_t);
    vel_out(i) = alpha * curr_v + (1 - alpha) * prev_v;
    
    prev_v = vel_out(i);
    prev_pos = curr_pos;
    prev_t = curr_t;
end
 
plot(t, vel_out,  'LineWidth', 2);

legend(["pos", "vel_{rpi}", "vel_{out}"]);

figure;
dt = diff(t);
plot(dt);
figure;
histogram(dt, 'Normalization', 'pdf');

%% udp stats

kitna_change = find(diff(pos) ~= 0);


%% pprz

data = csvread("pprz-log.csv", 1, 0);
tms = 0;
data(:, end+1) = zeros(size(data,1), 1);
for i = 1:1:size(data, 1)
    tm = 1.0/512;
    tms = tms + tm;
    data(i, end) = tms;
end