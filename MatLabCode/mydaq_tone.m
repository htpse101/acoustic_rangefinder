function [tone, mod_tone] = mydaq_tone(fs, input_tone)
    % function mod_tone = filter_tone(tone)
    % This filters a tone and returns the result
    % Written by Nephi Smith 9 Nov 2017
    amp = 0.9;
    input_tone = input_tone * amp;
    [output, time] = run_mydaq(fs, input_tone);
    tone = output(2, :);
    mod_tone = output(1, :);
end
