function coeff = WheelchairMotionPlanning(ic,fc,ts,tf)

A = [1 ts ts^2 ts^3;
    1 tf tf^2 tf^3; 
    0 1 2*ts 3*ts^2;
    0 1 2*tf 3*tf^2];

q = [ic(1); fc(1); ic(2); fc(2)];

coeff = inv(A) * q;

end