%% define function and gradient
syms x y
f = 5.*log10((sqrt((x - 1)^2 + (y - 6)^2))) - 1.25.*log10((sqrt((x + 1.33)^2 + (y - 4.5)^2))) - 1.25.*log10((sqrt((x - 2)^2 + (y - 3)^2))) - 1.25.*log10((sqrt((x - 2)^2 + (y - 6)^2)));
g = gradient(f, [x, y]);

%%
pos = [2; 0];

% Initialize theta and ROS stuff
sub_bump = rossubscriber('/bump');
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);
w = 0.2;
d = 0.2413;
theta = pi/2;
vR = -w*(d/2);
vL = w*(d/2);

while 1
    bumpMessage = receive(sub_bump);
    if any(bumpMessage.Data)
        msg.Data = [0, 0];
        send(pub, msg);
        break;
    end
    
    % Store current point
    pCurr = pos;
    
    % Calculate Gradient
    grad = -double(subs(g,[x, y],{pCurr(1),pCurr(2)}));
    
    % Get Gradient Direction and amount to turn
    gradDirection = atan2(grad(2),grad(1)) + 2*pi.*(grad(2) < 0);
    toTurn = gradDirection - theta;
    
    
    if (toTurn <= 0)
        vR = -w*(d/2);
        vL = w*(d/2);
    else
        vR = w*(d/2);
        vL = -w*(d/2);
    end
        
    
    % Turn NEATO
    msg.Data = [vL,vR];
    send(pub,msg);
    pause(abs(toTurn)/w);
    msg.Data = [0,0];
    send(pub,msg);
    
    % Update heading
    theta = gradDirection;
    
    % Calculate next point
    step = grad./norm(grad);
    pos = pos + 0.4*step;
    
    % Calculate distance
    distance = sqrt(sum((pos - pCurr).^2));
    
    % Drive NEATO fowards and stop
    msg.Data = [0.15, 0.15];
    send(pub,msg);
    pause(distance/0.5);
    msg.Data = [0,0];
    send(pub,msg);
end

msg.Data = [0,0];
send(pub,msg);