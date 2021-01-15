
% ignore the blackbox headers
data_in = readmatrix("latency-msp-rpi.BFL.csv", 'NumHeaderLines', 109);

% only select time and rcCommands
% "loopIteration","time","axisP[0]","axisP[1]","axisP[2]","axisI[0]","axisI[1]","axisI[2]","axisD[0]","axisD[1]","axisF[0]","axisF[1]","axisF[2]","rcCommand[0]","rcCommand[1]","rcCommand[2]","rcCommand[3]","setpoint[0]","setpoint[1]","setpoint[2]","setpoint[3]","vbatLatest","amperageLatest","BaroAlt","gyroADC[0]","gyroADC[1]","gyroADC[2]","accSmooth[0]","accSmooth[1]","accSmooth[2]","debug[0]","debug[1]","debug[2]","debug[3]","motor[0]","motor[1]","motor[2]","motor[3]","flightModeFlags","stateFlags","failsafePhase","rxSignalReceived","rxFlightChannelsValid","heading[0]","heading[1]","heading[2]","axisSum[0]","axisSum[1]","axisSum[2]","rcCommands[0]","rcCommands[1]","rcCommands[2]","rcCommands[3]","axisError[0]","axisError[1]","axisError[2]"
data = data_in(:, [1,2,14:17]);

%% check jitter and latency if any on timestamps

% remove inactive durations
rm_idx = find(data(:,6) == 1000);

% delete inactive rows
data(rm_idx, :) = [];

% microseconds to seconds
t = data(:,2) * (10)^(-6);

figure();
subplot(2,2,1);
plot(t);
xlabel("samples");
ylabel("seconds");

subplot(2,2,2);
dt = diff(t);
plot(dt);
xlabel("samples");
ylabel("\Delta t (s)");
ylim([0 4] * 10 ^ (-4));

subplot(2,2,3);
% get delay (1st bin is sampling time)
h = histogram(delta_t, 'Normalization', 'pdf');
hold on;
plot(h.BinEdges(2:end), h.Values, 'or', 'MarkerSize', 20)
xlabel("\Delta t bins (s)");
ylabel("samples");
% 0 seconds to 50 ms
xlim([0, 0.05]);

subplot(2,2,4);
freq_per_sample = 1.0./dt;
boxchart(freq_per_sample, 'MarkerStyle', 'None');
ylim([7000 9000]);
ylabel("freq (Hz)");

%% check thrust jitter and latency..
thrust = data(:,6);

% find indices at which thrust was incremented
idx = find(diff(round(thrust)) > 0); 

% find time at those indices
thrust_changed_at = t(idx);

% find how often thrust is changed
delta_t = diff(thrust_changed_at);
fprintf("freq of thurst commands: %f Hz\n", 1.0/mean(delta_t));

freq_thrust = 1.0./delta_t;
boxchart(freq_thrust);
ylabel("freq (Hz)");
ylim([20 80]);

%% fft try..

Y = fft(thrust);
L = length(Y);
Fs = 1 / mean(dt);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
