
x1 = get(x_car);
x1=x1.Data;

x2 = get(x_ref);
x2=x2.Data;

y1 = get(y_car);
y1=y1.Data;

y2 = get(y_ref);
y2=y2.Data;

figure(20)
plot(x1,y1,x2,y2)
% daspect([1 1 1])
title('Desired Trajectory (Red) - Vehicle Trajectory (Blue)')
ylabel('Y[m]')
xlabel('X[m]')
