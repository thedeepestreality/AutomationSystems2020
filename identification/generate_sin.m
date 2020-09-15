function out = generate_sin(t)
w = ceil(t/1);
s = sin(2*pi*t*w);
out = [s,w];