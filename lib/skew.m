function my_skew = skew(omega)
% Calculates the skew-symmetric matrix
% skew(omega)
% Where omega is a 3x1 column vector, w = [w1;w2;w3], such that the out put is:
% skew-symetric matrix = [0 -w3 w2; w3 0 w1; -w2 w1 0]

my_skew = [0 -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];

end