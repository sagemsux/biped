function walk(t,y,r,L,O)

    for i=1:length(t)
        clf
        [A,B,C,D] = kinematics([y(i,1),y(i,2),y(i,3)],r,L,O);
        plotbot(A,B,C,D);
        drawnow
        pause(0.00001)
    end


end