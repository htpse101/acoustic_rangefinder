function [tone, mod_tone] = filter_tone(fs, tone)
    % function mod_tone = filter_tone(tone)
    % This filters a tone and returns the result
    % Written by Nephi Smith 9 Nov 2017
%     mod_tone = filter(test_filter_2, tone);
    % This works
    
    mod_tone = tone;
    len_t = length(tone);
%     mod_tone = tone + a * rand(1, len_t);
    % end
    mod_tone = mod_tone + 0.05*mod_tone.^2;
    mod_tone = tone * -1;
%     linear filter
    mod_tone = mod_tone * 0.5;
%     f = linspace(0.5, 1, len_t);
%     mod_tone = mod_tone .* f;
    
    % add noise
    a = 0.01;
%     mod_tone = mod_tone + a * rand(1, len_t);
    %     a = 0.0;
    
%     idx = round(len_t * 0.8);
%     tone2 = [zeros(1, len_t - idx), tone(1:idx)];
%     mod_tone = tone2; % + a * rand(1, len_t);
    
end
