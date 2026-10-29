function x_cross = mmav_find_crossing(x, y, target)
%MMAV_FIND_CROSSING First linear crossing of y(x)=target.
if nargin < 3
    target = 0;
end
x = x(:); y = y(:);
yr = y - target;
x_cross = NaN;
for k = 1:numel(x)-1
    if ~isfinite(yr(k)) || ~isfinite(yr(k+1))
        continue;
    end
    if yr(k) == 0
        x_cross = x(k);
        return;
    end
    if yr(k)*yr(k+1) < 0 || yr(k+1)==0
        x_cross = interp1(y(k:k+1), x(k:k+1), target, 'linear');
        return;
    end
end
end
