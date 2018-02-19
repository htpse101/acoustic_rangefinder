function tone = make_tone(freq, duration, fs)
    % function tone = make_tone(freq, duration, fs)
    % This function creates a sine wave at a given frequency for a given
    % length of time (duration) in seconds. The sampling rate is fs.
    % Written by Nephi Smith 2 Nov 2017
%     pp_sw = round(fs/freq) + 1;
%     points_sine = sin(linspace(0, 2*pi, pp_sw));
%     points_sine = points_sine(1:length(points_sine) - 1);
%     num_sine_waves = round(duration * fs / length(points_sine));
%     tone = zeros(1, duration*fs);
%     signal = kron(ones(1, num_sine_waves), points_sine);  
%     min_idx = min([length(signal), duration*fs]);
%     tone(1:min_idx) = signal(1:min_idx);  % force same size array
%     wanted_power = 0.7071;
%     tone = tone * (wanted_power/rms(tone));
    
    
%     fs = 45000;
%     dur = 0.005; % smallest duration for 500 hz is 0.002 seconds
%     freq = 1000;
%     duration = 0.01;
    n_points = round(duration*fs);
% %     pp_sw = round(fs/freq) + 1;
% %     points_sine = sin(linspace(0, 2*pi, pp_sw));
% % %     signal = sin(2*pi*freq*
% %     points_sine = points_sine(1:length(points_sine) - 1);
% %     num_sine_waves = 2 * round(duration * fs / length(points_sine));
% % %     tone = zeros(1, duration*fs);
% %     signal = kron(ones(1, num_sine_waves), points_sine);  
    n = 1:n_points;
    tone = sin(2 * pi * freq * n / fs);  % borrowed from https://stackoverflow.com/questions/25658052/why-doesnt-matlab-give-me-an-8khz-sinewave-for-16khz-sampling-frequency
    
    
%     tone = signal(1:n_points);
    tone = tone/max(tone);
end
