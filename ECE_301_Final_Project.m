% 301 Final Project - Pan-Tompkins QRS Detection via Arduino
% MATLAB Side: Sends ECG samples over serial, receives QRS detections back,
% and plots all pipeline stages + detected peaks + RR intervals.

clear; clc; close all;

% Settings
COM_PORT   = 'COM4';
BAUD       = 115200;
RECORD     = '104';
NUM_SAMP   = 3000;
FS         = 360;

% Load MIT-BIH data
fprintf('Loading MIT-BIH record %s...\n', RECORD);
[sig, Fs, tm] = rdsamp(RECORD, [], NUM_SAMP);
signal = sig(1:NUM_SAMP, 1);
tm     = tm(1:NUM_SAMP);


fprintf('Running MATLAB reference pipeline...\n');

% MATLAB implementation of Pan-Tompkins QRS detection
[b, a]   = butter(2, [5 15] / (Fs/2), 'bandpass');
filtered = filtfilt(b, a, signal);

%Pan tompkins steps: derivative, squaring, moving window integration
deriv      = diff(filtered); deriv = [deriv; 0];
squared    = deriv .^ 2;
win_size   = round(0.15 * Fs);
integrated = movmean(squared, win_size);

% Adaptive thresholding for peak detection (different than Arduinos)
k         = 0.1 * (max(integrated) / std(integrated));
threshold = mean(integrated) + k * std(integrated);
[pks, locs] = findpeaks(integrated, ...
    'MinPeakHeight',   threshold, ...
    'MinPeakDistance', round(0.15 * Fs));

% Calculate RR intervals and identify abnormalities (>20% deviation from mean)
RR      = diff(locs) / Fs;
RR_time = tm(locs(2:end));
rr_mean = mean(RR);
abnormal = abs(RR - rr_mean) > 0.2 * rr_mean;

% Open serial port
fprintf('Opening serial port %s at %d baud...\n', COM_PORT, BAUD);
s = serialport(COM_PORT, BAUD);
s.Timeout = 10;
configureTerminator(s, 'LF');
flush(s);

% Wait for READY signal from Arduino
fprintf('Waiting for Arduino READY signal...\n');
ack = '';
t_wait = tic;
while ~contains(ack, 'READY')
    if toc(t_wait) > 10
        error('Arduino did not send READY within 10 s. Check flashing/wiring.');
    end
    try
        ack = strtrim(readline(s));
    catch
    end
end
fprintf('Arduino ready.\n');
flush(s);

% Scale signal to int16 for ATmega integer arithmetic
% SCALE = 1000 keeps values in a comfortable range for the filter chain.
% Do NOT use 32767 — it causes overflow after squaring
SCALE     = 1000;
sig_norm  = signal / max(abs(signal));
sig_int16 = int16(sig_norm * SCALE);

% Send samples, receive QRS signals
qrs_arduino = false(NUM_SAMP, 1);

fprintf('Sending %d samples to Arduino...\n', NUM_SAMP);
t_start = tic;

for i = 1:NUM_SAMP
    writeline(s, sprintf('%d', sig_int16(i)));

    resp = '';
    try
        resp = strtrim(readline(s));
    catch
        resp = 'N';
    end

    if strcmp(resp, 'Q')
        qrs_arduino(i) = true;
    end

    if mod(i, round(NUM_SAMP/10)) == 0
        fprintf('  %d%% complete (%.1f s elapsed)\n', ...
            round(100*i/NUM_SAMP), toc(t_start));
    end
end

fprintf('Transfer complete in %.1f s (%.1f samples/s).\n', ...
    toc(t_start), NUM_SAMP/toc(t_start));
clear s;

% Extract Arduino detections
arduino_locs = find(qrs_arduino);
if isempty(arduino_locs)
    warning('NO QRS DETECTIONS! NOOOOOOOOOOO');
end

if numel(arduino_locs) > 1
    RR_ard       = diff(arduino_locs) / Fs;
    RR_time_ard  = tm(arduino_locs(2:end));
    rr_mean_ard  = mean(RR_ard);
    abnormal_ard = abs(RR_ard - rr_mean_ard) > 0.2 * rr_mean_ard;
end

%-------------- Plots -------------
ratio = 1:NUM_SAMP;

%plot all stage of Pan-Tompkins pipeline + detected peaks + RR intervals
figure('Name','ECG Pipeline');
subplot(5,1,1); plot(tm, signal);          title('Original ECG');           xlabel('Time (s)'); ylabel('Amplitude');
subplot(5,1,2); plot(tm, filtered);        title('Band-Pass (5-15 Hz)');    xlabel('Time (s)'); ylabel('Amplitude');
subplot(5,1,3); plot(tm, deriv);           title('Derivative');             xlabel('Time (s)'); ylabel('Amplitude');
subplot(5,1,4); plot(tm, squared);         title('Squared');                xlabel('Time (s)'); ylabel('Amplitude');
subplot(5,1,5); plot(tm, integrated);
yline(threshold,'r--','Threshold');        title('MWI');                    xlabel('Time (s)'); ylabel('Amplitude');

% Overlay detected peaks on integrated signal for both MATLAB and Arduino
figure('Name','MATLAB QRS Detection');
plot(tm, integrated); hold on;
plot(tm(locs), pks, 'ro', 'MarkerSize', 8, 'DisplayName', 'MATLAB QRS');
yline(threshold, 'k--', 'Threshold');
legend; title('MATLAB Pan-Tompkins'); xlabel('Time (s)');

%Arduino QRS detections
figure('Name','Arduino QRS Detection');
plot(tm, signal); hold on;
if ~isempty(arduino_locs)
    plot(tm(arduino_locs), signal(arduino_locs), ...
        'rv', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Arduino QRS');
end
legend; title('Arduino Pan-Tompkins (on raw signal)'); xlabel('Time (s)');

% Overlay MATLAB and Arduino detections on integrated signal
figure('Name','MATLAB vs Arduino Overlay');
plot(tm, integrated, 'b', 'DisplayName', 'Integrated'); hold on;
plot(tm(locs), pks, 'go', 'MarkerSize', 8, 'DisplayName', 'MATLAB peaks');
if ~isempty(arduino_locs)
    plot(tm(arduino_locs), integrated(arduino_locs), ...
        'rv', 'MarkerSize', 8, 'DisplayName', 'Arduino peaks');
end
yline(threshold, 'k--', 'MATLAB threshold');
legend; title('MATLAB vs Arduino Comparison'); xlabel('Time (s)');

%RR intervals with abnormalities highlighted
figure('Name','RR Intervals');
subplot(2,1,1);
plot(RR_time, RR, 'b.-'); hold on;
plot(RR_time(abnormal), RR(abnormal), 'ro', 'MarkerSize', 8);
title('MATLAB RR Intervals (red = abnormal >20%)'); xlabel('Time (s)'); ylabel('RR (s)');

subplot(2,1,2);
if numel(arduino_locs) > 1
    plot(RR_time_ard, RR_ard, 'b.-'); hold on;
    plot(RR_time_ard(abnormal_ard), RR_ard(abnormal_ard), 'ro', 'MarkerSize', 8);
end
title('Arduino RR Intervals (red = abnormal >20%)'); xlabel('Time (s)'); ylabel('RR (s)');

% Summary of expiremental results
fprintf('\n========== Detection Summary ==========\n');
fprintf('MATLAB  QRS detected : %d\n', numel(locs));
fprintf('Arduino QRS detected : %d\n', numel(arduino_locs));
if numel(locs) > 1
    fprintf('Average HR (MATLAB)  : %.1f BPM\n', 60/mean(RR));
end
if numel(arduino_locs) > 1
    fprintf('Average HR (Arduino) : %.1f BPM\n', 60/mean(RR_ard));
end
fprintf('=======================================\n');