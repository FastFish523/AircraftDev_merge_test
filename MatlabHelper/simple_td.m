function [x1, x2] = simple_td(v, x1_prev, x2_prev, r, h, dt)
    d = r * h;
    d0 = h * d;
    y = x1_prev - v + h * x2_prev;
    
    a0 = sqrt(d*d + 8*r*abs(y));
    
    if abs(y) > d0
        a = x2_prev + (a0 - d)/2 * sign(y);
    else
        a = x2_prev + y / h;
    end
    
    fhan = -r * sign(a) * min(abs(a), d) / d;
    
    % 2. 状态更新
    x2 = x2_prev + fhan * dt;
    x1 = x1_prev + x2 * dt;
end