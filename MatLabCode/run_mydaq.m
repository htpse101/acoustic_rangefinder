function [output_data, time] = run_mydaq(fs, input_data)
    % function [output_data, time] = run_mydaq(fs, input_data)
    % This runs the MyDAQ with the given input data at the given sampling
    % frequency (fs). The result is returned as a vector output_data with
    % the time vector.
    % Written by Nephi 10 Oct 2017
    
    % d = daq.getDevices();
    s = daq.createSession('ni');
    s.Rate = fs;  % 100,000 scans per second
    s.release();
    % Added input channel
    dev_name = 'myDAQ1';
    addAnalogInputChannel(s, dev_name, 'ai0', 'Voltage');
    addAnalogInputChannel(s, dev_name, 'ai1', 'Voltage');

    % Added output channel
    addAnalogOutputChannel(s, dev_name, 'ao0', 'Voltage');

    % Queue data to send
    queueOutputData(s, input_data')
    
    % Execute operation
    [output_data, time] = s.startForeground();
    output_data = output_data';
end