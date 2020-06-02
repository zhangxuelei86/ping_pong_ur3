%% Test for theta of incoming robot

tr  = robot.model.fkine(robot.model.getpos());

r = tr2rpy(tr);
r(3) = r(2) + pi;

R = rpy2tr(r);

tr = tr * R;

trplot(tr);
m = [1; 1; 1; 1; 1; 1];
q = robot.model.ikine(tr,robot.model.getpos());
robot.model.animate(q);