%Cinematica directa

clc;
clear;

%% Variables para el calculo simbolico
syms q1 q2 q3 q4 l1 l2 l3 l4 p;

A1 = [cos(q1), 0, sin(q1),   0;
      sin(q1), 0, -cos(q1),  0;
            0, 1,        0,  l1;
            0, 0,        0,   1];

A2 = [cos(q2 + p/2), -sin(q2 + p/2), 0, l2*cos(q2 + p/2);
      sin(q2 + p/2),  cos(q2 + p/2), 0, l2*sin(q2 + p/2);
                  0,              0, 1,                0;
                  0,              0, 0,                1];

A3 = [cos(q3 - p/2),  0, -sin(q3 - p/2), l3*cos(q3 - p/2);
      sin(q3 - p/2),  0,  cos(q3 - p/2), l3*sin(q3 - p/2);
                  0, -1,              0,                0;
                  0,  0,              0,                1];

A4 = [cos(q4), -sin(q4), 0,   0;
      sin(q4),  cos(q4), 0,   0;
            0,        0, 1,  l4;
            0,        0, 0,   1];

AT = simplify(A1*A2*A3*A4);