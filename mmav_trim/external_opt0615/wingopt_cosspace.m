function eta = wingopt_cosspace(N)
%WINGOPT_COSSPACE  Cosine-spaced points in [0..1] (clusters near 0 and 1).
%
% N>=2 recommended.

if N < 2
    eta = 0;
    return;
end

i = (0:(N-1)).';
eta = 0.5 * (1 - cos(pi * i / (N-1)));

end
