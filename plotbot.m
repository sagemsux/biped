function plotbot(A,B,C,D)
    clf
    plot([A(1) C(1) B(1) C(1) D(1)],[A(2) C(2) B(2) C(2) D(2)],'b','LineWidth',1.5)
    hold on
    plot([-100 100],[0 0],'k','LineWidth',2) % plotting ground as a black line
    axis([-1 5 -0.1 3 ]);
end