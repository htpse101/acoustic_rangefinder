



complex_array = zeros(1, 2*length(mod_signal));
for i=1:(length(mod_signal))
   complex_idx = i;
   real_idx = 2*(i-1) + 1;
   im_idx = 2*(i-1) + 2;
   complex_array(1, real_idx) = real(mod_signal(i));
   complex_array(1, im_idx) = imag(mod_signal(1));
end

x = 2;


