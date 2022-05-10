% Function to navigate f(x, y) & gradient f(x, y)

% Set script parameter values (robot wheel base, angular speed, linear
% speed, gradient ascent parameters like lambda)
d = 0.235;
omega = 0.2;
speed = 0.75;
lambda = 0.01;

% Coordinate parameters
syms x y;

% Equation containing objects
f = - 2*log(sqrt((x-1.0).^2 + (y+0.7).^2)) - 2*log(sqrt((x+0.25).^2 + (y+1.0).^2))  - 2*log(sqrt((x-1.41).^2 + (y+2).^2)) + 15*log(sqrt((x-0.75).^2 + (y+2.5).^2));

% Gradeint descent
g = -gradient(f, [x, y]);

% Initialize robot position & heading
position = [0; 0];
heading = [1; 0];

pub = rospublisher('raw_vel');

% Stop robot in case it's moving
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

% Place the robot in the initial orientation
placeNeato(position(1), position(2), heading(1), heading(2));

% Wait a bit for placement
pause(2);

% While loop; computes angle to turn the robot to align it to the gradient
% Turns the robot
% Drives the robot forwards by lambda * sqrt(gradient^2)
% Checks if close enough to stop
msg = rosmessage(pub);
start = rostime('now');
stop = false;
while ~stop
    currentGradient = double(subs(g, {x, y}, {position(1), position(2)}));
    crossP = cross([heading; 0], [currentGradient; 0]);
    turnTheta = asin(norm(crossP)/(norm(heading)*norm(currentGradient)));
    turnTime = double(turnTheta)/omega;
    V_L = -sign(crossP(3))*omega*d/2;
    V_R = sign(crossP(3))*omega*d/2;
    msg.Data = [V_L, V_R];
    send(pub, msg);

    start = rostime('now');
    while rostime('now')-start < turnTime
        pause(0.01)
    end

    heading = currentGradient;
    
    forward = norm(currentGradient*lambda);
    forwardTime = forward/speed;
    msg.Data = [speed, speed];
    send(pub, msg);

    start = rostime('now');
    while rostime('now')-start < forwardTime
        pause(0.01)
    end
    
    position = position + currentGradient*lambda;
    
    stop = (forward < 0.01);

% Makes sure the robot is stopped.
msg.Data = [0, 0];
send(pub, msg);

end


