% Start with a blank rigid body tree model.
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',4);
%%
% Specify arm lengths for the robot arm.
L1 = 0.95;

%%
% Add |'link1'| body with |'joint1'| joint.
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');
%%
% Add |'tool'| end effector with |'fix1'| fixed joint.
body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L1, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link1');

%%
% Show details of the robot to validate the input properties. The robot
% should have two non-fixed joints for the rigid bodies and a fixed body
% for the end-effector.
showdetails(robot)

%% Define Object
% Define circle of bouncy object
% is in the _xy_ plane with a radius of 0.95.
x_circle = 0;
y_circle = 0;
r_circle= 0.95;
th = 0:pi/100:2*pi;
a_circle = r_circle*cos(th)+x_circle;
b_circle = r_circle*sin(th)+y_circle;


%% Time counter
t = (0:0.2:30)'; % Time
count = length(t);

%% Define Jacobian Matrix
%y_pos =equation of postion y, dy_pos is velocity, ddy_pos is acceleration
%q is angle, dq is change in angle
q = zeros(count,1);
dq = zeros(count,1);

Torq_finger = zeros(count,1);

dyx_prev = zeros(count,1);
y_pos = zeros(count,1);
dy_pos = zeros(count,1);
ddy_pos = zeros(count,1);
yd(1) = 0; %equilibrium
q(1) = 0;
dq(1) = 0;
Md = 4;
Bd = 20;
Kd = 15;
Torq_finger(1:20) = -6;
Torq_finger(21:30) = -3;
dyx_prev(1) = 0;
T_samp = 0.2;
y_pos(1) = L1*sin(q(1));
dy_pos(1) = (L1*cos(dq(1)))*dq(1);
x_pos = zeros(count,1);
x_pos(1) = L1*cos(q(1));

for i = 2:count


    %% Define impedance control

        %y_diff = y_pos- yd, yd is equlibrium point
        y_diff = y_pos(i-1)- yd;
        % since dyd should be ideally zero, dy_diff = dy__pos
        dy_diff = dy_pos(i-1);
        %Torq_finger = Md*ddy_diff+Bd*dy_diff+Kd*y_diff; Torque finger is the force exerted by finger
        ddy_diff = (1/Md)*(Torq_finger(i-1)-Bd*dy_diff-Kd*y_diff);
        %T_samp is sampling period
        ddy_pos(i-1) = ddy_diff;
        dy_pos_new = dy_pos(i-1) + ddy_diff*T_samp;
        dy_pos(i,:) = dy_pos_new;
        y_pos(i,:) = y_pos(i-1)+dy_pos(i-1)*T_samp + ddy_diff*T_samp*T_samp;
        %q(i,:)  = q(i-1) + dq(i);
        q(i,:)  = asind(y_pos(i)/L1);
        dq(i,:) = (q(i)-q(i-1))/T_samp;
        x_pos(i,:) = L1*cosd(q(i));
  
end
%% Inverse Kinematics
%q0 = homeConfiguration(robot);
%ndof = length(q0);
%qs = zeros(count, ndof);

%ik = inverseKinematics('RigidBodyTree', robot);
%weights = [0, 0, 0, 1, 1, 0];
%endEffector = 'tool';

%%Graph here
%show(robot);
figure(1);
view(2);

plot(t,y_pos,'k');
hold on;
plot(t,dy_pos,'g');
hold on;
plot(t,ddy_pos,'r');
hold on;
plot(t,q,'c');
hold on;
plot(t,x_pos,'m');
hold on;

%% Animate The Solution
% Plot the robot for each frame of the solution using that specific robot 
% configuration. Also, plot the desired trajectory.
%qInitial = q0; % Use home configuration as the initial guess
figure(2);
ct = 1;
for i = 1:count
    
    plot ( [0 x_pos(i,:)],[0 y_pos(i,:)], 'linewidth',2);
    grid on
    axis([-0 1 -1 1]);
    pause(0.1);
    m(ct) = getframe(gcf);
    ct = ct+1;
end
movie(m);
videofile = VideoWriter ('impedance control','uncompressed AVI')
open(videofile);
%%
% Show the robot in the first configuration of the trajectory. Adjust the 
% plot to show the 2-D plane that circle is drawn on. Plot the desired 
% trajectory.



%show(robot,qs(1,:)');
%ax = gca;
%ax.Projection = 'orthographic';
%hold on;
%plot(a_circle,b_circle,'k');
%axis([-1 1 -1 1]);%%set the x&y axis 

%%
% Set up a <docid:robotics_ref.mw_9b7bd9b2-cebc-4848-a38a-2eb93d51da03 Rate> object to display the robot 
% trajectory at a fixed rate of 15 frames per second. Show the robot in
% each configuration from the inverse kinematic solver. Watch as the arm
% traces the circular trajectory shown.

%framesPerSecond = 1;
%r = rateControl(framesPerSecond);

%for i = 1:3
   % for i = 1:count
      % show(robot,qs(1,:)','PreservePlot',false);
      % drawnow
     % waitfor(r);
 %   end
%end