function y = moving_avg(u)
persistent prev;
if (isempty(prev))
   prev = u;
   y = u/2;
   return;
end

% y = (u(i-1)+u(i))/2;

y = (prev + u)/2;
prev = u;

