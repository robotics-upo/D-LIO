function vo = quatconv(q, vi)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
vo = quatrotate(quatinv(q), vi);
end

