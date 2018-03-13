% Conv Test
A = [0,0,1,5,2,0,0,0,0,0]; %Start at 16
B = [2, 5, 1];

c = conv(A,B);
plot(c);
[v, idx] = max(c);
idx_1 = idx - length(B) + 1;
%t = 2*-1+dt*(0:(length(u)-1));
