function [A,B,C,D] = kinematics(angles,r,L,O)
    
    th1 = angles(1);
    th2 = angles(2);
    th3 = angles(3);
    
    A = O;
    z1 = A(1);
    z2 = A(2);
    
    B = [z1+r*sin(th1)-r*sin(th2),z2+r*cos(th1) - r*cos(th2)];
    
    C = [z1+r*sin(th1),z2+r*cos(th1)];
    
    D = [z1+r*sin(th1)+L*sin(th3),z2+r*cos(th1)+L*cos(th3)];
    
end