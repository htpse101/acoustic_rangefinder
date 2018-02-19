function new_signal = time_pad_signal(signal_time, signal_start_fraction, fs, signal)
    % function new_signal = time_pad_signal(signal_time, signal_start_fraction, fs, signal)
    % This function wraps a signal with zeros. This allows it to be sent
    % with lots of zeros before and after. A user can set the signal_time
    % to specify how long the signal will be sent for. Then they can set
    % the signal_start_fraction to specify at what fraction of that time
    % the signal should be sent. fs = sampling frequency, signal = time
    % signal.
    % Written by Nephi Smith 18 Oct 2017

    total_points = fs * signal_time;
    signal_idx = round(total_points*signal_start_fraction);
    x = zeros(1, signal_idx - 1);
    y = signal;
    z = zeros(1, total_points - (signal_idx - 1 + length(signal)));
    %     a = length(x) + length(y) + length(z);
    new_signal = [x y z];  %; x y z
end