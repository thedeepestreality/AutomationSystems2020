function y = arma_1(u)
persistent buf;
if (isempty(buf))
    buf = zeros(1,2);
end
y = (u + 2*buf(1) + buf(2))/4;
buf = [y buf(1)];

