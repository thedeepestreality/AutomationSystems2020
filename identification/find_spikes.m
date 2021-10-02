function da = find_spikes(c)
    y = diff(c(:,2));
    sg = sign(y(1));
    y_spikes = [];
    for i = 1:length(y)
        if sg ~= sign(y(i))
            y_spikes = [y_spikes; c(i-1,:)];
        end
        sg = sign(y(i));
    end
        
    u = diff(c(:,1));
    sg = sign(u(1));
    u_spikes = [];
    for i = 1:length(u)
        if sg ~= sign(u(i))
            u_spikes = [u_spikes; c(i-1,:)];
        end
        sg = sign(u(i));
    end
    
    y_max = mean(y_spikes(y_spikes(:,2)>0,2));
    y_min = mean(y_spikes(y_spikes(:,2)<0,2));
    
    u_max = mean(u_spikes(u_spikes(:,1)>0,1));
    u_min = mean(u_spikes(u_spikes(:,1)<0,1));
    A_y = (y_max-y_min)/2;
    A_u = (u_max-u_min)/2;
    da = A_y/A_u;
    end
