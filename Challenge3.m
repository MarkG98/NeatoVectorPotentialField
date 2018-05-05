% sub = rossubscriber('/stable_scan');
% scan_message = receive(sub);
% r = scan_message.Ranges(1:end-1);
% theta = [0:359];

%Initialize theta and ROS stuff
sub_bump = rossubscriber('/bump');
sub = rossubscriber('/stable_scan');
pub = rospublisher('/raw_vel');
msg = rosmessage(pub);

w = 0.2;
d = 0.2413;
vR = -w*(d/2);
vL = w*(d/2);

bucketPos = [1;6];
pos = [0; 0];
heading = pi/2;

% Initialize syms
syms x y;

while 1
    bumpMessage = receive(sub_bump);
    if any(bumpMessage.Data)
        msg.Data = [0, 0];
        send(pub, msg);
        break;
    end
    
    % Store current point
    pCurr = pos;
    
    % Get scan and translate to global
    scan_message = receive(sub);
    r = scan_message.Ranges(1:end-1);
    theta = [0:359];
    
    [xb, yb, inliers, outliers] = robustLineFit(r, theta, 0.017, 1100, 15);
    
    ind = 1;
    points =[];
    for line = 1:length(xb)
        for pt = 1:size(xb{line},2)
            points(ind,:) = [xb{line}(pt) yb{line}(pt)];
            ind = ind + 1;
        end
    end
     
    [xg, yg] = toGlobal(points,pCurr,heading,bucketPos);
    pointsg = [xg; yg];
    
    % Establish function and gradient
    f = 12.*log10((sqrt((x - 1)^2 + (y - 6)^2)));
    for spot = 1:size(pointsg,2)
        f = f - .1.*log10((sqrt((x - pointsg(1,spot))^2 + (y - pointsg(2,spot))^2)));
    end
    g = gradient(f, [x, y]);
    
    % Calculate Gradient
    grad = -double(subs(g,[x, y],{pCurr(1),pCurr(2)}));
    
    % Get Gradient Direction and amount to turn
    gradDirection = atan2(grad(2),grad(1)) + 2*pi.*(grad(2) < 0);
    toTurn = gradDirection - heading;
    
    
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
    heading = gradDirection;
    
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

% [xb, yb, inliers, outliers] = robustLineFit(r, theta, 0.017, 7000, 15);
% 
% ind = 1;
% for line = 1:length(xb)
%     for pt = 1:size(xb{line},2)
%         points(ind,:) = [xb{line}(pt) yb{line}(pt)];
%         ind = ind + 1;
%     end
% end
% 
% [x, y] = toGlobal(points,[0 4],pi/2,[1;6]);
% 
% clf
% hold on
% plot(x, y,'*');
% axis equal
% hold off
%% FUNCTIONS
%-------------------------RANSAC--------------------------
function [xbS, ybS, inliersS, outliersS] = robustLineFit(r, theta, d, n, density)
    [x, y] = PolToCart(r, theta);
    points = [x, y];
    c = 1;
    
    while 1
        maxIn = 0;
        for w = 1:n
            [Lhat, Nhat, point1, point2] = getLine(points(:,1), points(:,2));
            [inliers, outliers] = InAndOuts(point1, points, d, Nhat);
            if size(inliers,1) > maxIn
                maxIn = size(inliers,1);
                inliersFinal = inliers;
                outliersFinal = outliers;
                LhatFinal = Lhat;
                point1Final = point1;
                point2Final = point2;
            end
        end
        
        % kill if too few inliers
        if size(inliersFinal,1) < 4
            break;
        end
        
        inliersFinal = sortrows(inliersFinal);
        
        Lfin = 2;
        start = 1;
        L = [];
        for p = 1:size(inliersFinal,1) - 1
            if (abs(dot(inliersFinal(p + 1,:),LhatFinal) - dot(inliersFinal(p,:),LhatFinal)) < 0.3)
                L(1:Lfin,:) = inliersFinal(start:p+1,:);
                Lfin = Lfin + 1;
                
                if (p + 1 == size(inliersFinal,1) && size(L,1) > 3)
                    dist = sqrt(sum((L(1,:) - L(end,:)).^2));
                    pts = density*dist;
                    u = linspace(0,dist,pts);
        
                    xb = L(1,1) + u.*LhatFinal(1);
                    yb = L(1,2) + u.*LhatFinal(2);
                
                    xbS{c} = xb;
                    ybS{c} = yb;
                    inliersS{c} = L;
                    outliersS{c} = outliersFinal;
                    c = c + 1;
                    
                    L = [];
                end
            elseif (size(L,1) > 3)
                dist = sqrt(sum((L(1,:) - L(end,:)).^2));
                pts = density*dist; 
                u = linspace(0,dist,pts);
        
                xb = L(1,1) + u.*LhatFinal(1);
                yb = L(1,2) + u.*LhatFinal(2);
                
                xbS{c} = xb;
                ybS{c} = yb;
                inliersS{c} = L;
                outliersS{c} = outliersFinal;
                c = c + 1;
                
                Lfin = 2;
                start = p + 1;
                L = [];
            else
                start = p + 1;
                L =[];
                Lfin = 2;
            end
        end
        
        ind = 1;
        for m = 1:size(points,1)
            for n = 1:size(inliersFinal,1)
                if points(m,:) == inliersFinal(n,:)
                    del(ind,1) = m;
                    ind = ind + 1;
                end
            end
        end
        points(del,:) = [];
        del = [];
    end
end

function [x, y] = PolToCart(r, theta)
    c = 1;
    for i = 1:length(r)
        if and(r(i) ~= 0, r(i) < 14)
            r_clean(c,:) = r(i);
            theta_clean(c,:) = theta(i);
            c = c + 1;
        end
    end
    x = r_clean.*cosd(theta_clean);
    y = r_clean.*sind(theta_clean);
end

function [Lhat, Nhat, point1, point2] = getLine(x, y)
    points = [x y];
    rand = randperm(size(points,1),2);
    
    point1 = points(rand(1),:);
    point2 = points(rand(2),:);
    
    if point1(1) < point2(1)
        Lhat = (point2 - point1)./norm(point2 - point1);
    else (point1(1) > point2(1));
        Lhat = (point1 - point2)./norm(point1 - point2);
    end
    Nhat = [Lhat(2) -Lhat(1)];
end

function [inliers, outliers] = InAndOuts(point1, points, d, Nhat)
    m = 1;
    n = 1;
    for i = 1:size(points,1)
        point = points(i,:);
        dist = point - point1;
        res = dot(Nhat,dist);

        if abs(res) < d
            inliers(m,:) = point;
            m = m + 1;
        else
            outliers(n,:) = point;
            n = n + 1;
        end
    end
end
%------------------RANSAC--------------------------------

function [x, y] = toGlobal(points, pos, angle, bucketPos)
    pos = pos.*0.3048;
    
    Rlidar(1,:) = points(:,1);
    Rlidar(2,:) = points(:,2);
    Rlidar(3,:) = ones(1,size(points,1));
    
    d = 0.1143;
    Ttoneato = [1 0 0; 0 1 -d; 0 0 1];
    Rneato = Ttoneato*Rlidar;
    
    Ttoglobal = [1 0 pos(1); 0 1 pos(2); 0 0 1];
    Rtoglobal = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
    Rglobal = Ttoglobal*Rtoglobal*Rneato;
    
    x = Rglobal(1,:);
    y = Rglobal(2,:);
    
    ind = 1;
    del = [];
    for pt = 1:length(x)
        if (sqrt(sum(([x(pt);y(pt)]) - bucketPos).^2) < 0.18)
            del(ind) = pt;
            ind = ind + 1;
        end
    end
    
    x(del) = [];
    y(del) = [];
end