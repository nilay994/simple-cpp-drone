% script for natnet packet jitter

close all; clear all; clc;

data = csvread("rpi-old-method.csv", 1);
t = data(:,1);
x = data(:,2);
y = data(:,3);
z = data(:,4);

figure(1);
plot(t, z, '.');
grid on; xlabel("t"); ylabel("z");
title("altitude");

figure(2);
subplot(3,1,1);
plot(t);
title("time");

subplot(3,1,2);
delta_t = diff(t);
plot(delta_t, '.'); hold on;
title("\Delta(t)");
% most of the times (75% of the times)
delta_t_mostly = prctile(delta_t, 75);
freq_mostly = 1.0 / delta_t_mostly;
fprintf("Freq of natnet timestamps: %.02f Hz\n", freq_mostly);
yline(delta_t_mostly, '-r', 'LineWidth', 5);

subplot(3,1,3);
h = histogram(delta_t, 'Normalization', 'pdf'); hold on;
plot(h.BinEdges(2:end), h.Values, 'or', 'MarkerSize', 20);
title("histogram of timestamps")


% results seperate LAN: 208.72 Hz for seperate network card on optitrack pc, and a
% router on a different network as optitrack cameras, PC connected to
% router on different network via LAN (jitter seems to be one clock slower,
% peaks on next bin of histogram)

% results seperate WiFi: 93.74 Hz for seperate network card on optitrack pc, and a
% router on a different network as optitrack cameras, PC connected to
% router on different network via WiFi (jitter seems to be random,
% no secondary peaks in histogram bins)

% results same LAN as cameras: 175.37 Hz for same network card as cameras
% on optitrack pc, and no router, just the switch below the monitor (jitter
%, but not so much as expected from the same network warning from optitrack)

% results same WiFi as cameras: 81.70 Hz (worst) for same network card as cameras
% on optitrack pc, the switch below the monitor goes to a wifi router
