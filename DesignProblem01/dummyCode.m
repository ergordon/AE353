syms j1 j2 j3 t real

A = [0 0 (j2-j3)/j1; 0 0 0; (j1-j2)/j3 0 0]
B = [1/j1 0; 0 1/j2; 0 0]
C = eye(3)
D = [0]

%%Zero Input
[V,F]=eig(A)