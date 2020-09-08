function y = arma_notch(u)
persistent buf_in buf_out;
if (isempty(buf_in))
    buf_in = zeros(1,3);
    buf_out = zeros(1,2);
end
buf_in = [u buf_in(1:end-1)];
b = [0.927040342731733  -1.499982783482296   0.927040342731733];
a = [-1.499982783482296   0.854080685463467];
y = dot(b,buf_in) - dot(a,buf_out);
buf_out = [y buf_out(1)];

