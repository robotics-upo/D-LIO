function [p_idx, s_idx] = combteeth(ta, tb, dtmax)
%UNTITLED Summary of this function goes here

%   Detailed explanation goes here
Ka  = length(ta);
Kb  = length(tb);

p_idx = ones(Ka, 1);
s_idx = ones(Ka, 1);

last_tb_idx = 1;

for k=1:Ka
    
    p_idx(k) = NaN;
    s_idx(k) = NaN;
    
    for n=last_tb_idx:Kb-1
        if tb(n) <= ta(k) && ta(k) < tb(n+1) && abs(tb(n+1) - tb(n)) < dtmax
            p_idx(k) = n;
            s_idx(k) = n+1;
            last_tb_idx = n;
            break;
        end
    end
end

end



