function [H, F] = f_analysis(x, fs)
    % function [H, F] = f_analysis(x, fs)    
    % This performs the frequency analysis for the input x and sampling
    % frequency fs.
    Npoints = 2*length(x);
    [H, F] = freqz(x, 1, Npoints, fs);
end