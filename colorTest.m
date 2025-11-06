brick = ConnectBrick("OOO");

ultrasoundPort = 1;
gyroPort = 2;

currentGrid = [1, 1];
wallArray = false(6, 3, 4);

brick.GyroCalibrate(gyroPort);

wallLength = 50;

angle = NaN;
t0 = tic;
while isnan(angle) && toc(t0) < 5   % poll up to 5s
    pause(0.05);
    angle = brick.GyroAngle(gyroPort);
end

brick.SetColorMode(3, 4);

brick.beep();

hasGreen = false;
function main(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength, hasGreen)
    wallArray = radar(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength);
    
    % Extract the list of wall values for the current grid
    wallList = squeeze(wallArray(currentGrid(1), currentGrid(2), :))';
    
    % Find zero positions
    zeroPositions = find(wallList == 0);
    if ~isempty(zeroPositions)
        if (wallList(2) == 0)
            direction = 2;
        elseif (wallList(1) == 0)
            direction = 1;
        elseif (wallList(4) == 0)
            direction = 4;
        elseif (wallList(3) == 0)
            direction = 3;
        end
        fprintf("i found a path :D\n");
    else
        fprintf("im stuck ;(\n");
    end

    display(direction)

    if direction == 1
        % facing the right direction
    
    elseif direction == 2
        right(brick, gyroPort);
    
    elseif direction == 3
        % Randomly pick right or left, and run it twice
        if randi(2) == 1
            right(brick, gyroPort);
            right(brick, gyroPort);
        else
            left(brick, gyroPort);
            left(brick, gyroPort);
        end
    
    elseif direction == 4
        left(brick, gyroPort);
    end

    forward(brick, ultrasoundPort, 4, wallLength, gyroPort);

    reAdjust(brick, gyroPort);

    % check color for manual
    color = detectColor(brick, 3);
    
    if color == "green" && hasGreen == false
        manual(brick);
        hasGreen = true;
    elseif color == "blue"
        fprintf("I have completed my objective");
        return;
    end

    main(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength, hasGreen);
end



function left(brick, gyroPort)
    startAngle = brick.GyroAngle(gyroPort);
    targetAngle = startAngle - 80;  % aiming for 90 from start

    angle = brick.GyroAngle(gyroPort);

    while angle > targetAngle
        angle = brick.GyroAngle(gyroPort);

        % Keep rotating
        brick.MoveMotor('A', 20);
        brick.MoveMotor('B', -20);
        pause(0.05);
    end

    brick.StopAllMotors('Brake');
end

function reAdjust(brick, gyroPort)
    % Rotate to the nearest multiple of 90°, accounting for reversed gyro direction

    Kp = 2;            % proportional gain
    minSpeed = 15;     % minimum speed to overcome friction
    maxSpeed = 40;     % maximum turn speed
    tol = 0.5;         % degrees tolerance
    maxSec = 3.0;      % safety timeout

    t0 = tic;
    while true
        ang = brick.GyroAngle(gyroPort);
        
        % Find nearest multiple of 90
        target = round(ang / 90) * 90;
        
        % Compute signed shortest difference
        err = mod(target - ang + 180, 360) - 180;
        
        % Invert direction because gyro orientation is reversed
        err = -err;

        if abs(err) <= tol || toc(t0) > maxSec
            break;
        end

        sp = max(minSpeed, min(maxSpeed, Kp * abs(err)));

        if err > 0
            % Turn one direction
            brick.MoveMotor('A', sp);
            brick.MoveMotor('B', -sp);
        else
            % Turn the opposite direction
            brick.MoveMotor('A', -sp);
            brick.MoveMotor('B', sp);
        end

        pause(0.03);
    end

    brick.StopAllMotors('Brake');
end

function right(brick, gyroPort)
    startAngle = brick.GyroAngle(gyroPort);
    targetAngle = startAngle + 80;  % aiming for -90 from start

    angle = brick.GyroAngle(gyroPort);

    while angle < targetAngle
        angle = brick.GyroAngle(gyroPort);

        % Keep rotating right
        brick.MoveMotor('A', -20);
        brick.MoveMotor('B', 20);
        pause(0.05);
    end

    brick.StopAllMotors('Brake');
end

function forward(brick, ultrasoundPort, distance, wallLength, gyroPort)
    %=== Base forward powers (negative = forward in your setup) ===
    baseLeft  = -50;   % motor A
    baseRight = -60;   % motor B

    %=== Control params ===
    Kp = 2.0;          % steering gain (try 1.0–4.0)
    Kd = 0.0;          % add small damping if oscillating, e.g., 0.5
    dt = 0.05;         % loop period (s)
    eps = 0.5;         % deadband in deg to ignore tiny gyro noise

    %=== 1) Determine gyro sign robustly (left pulse) ===
    gyroSign = calibrateGyroSign(brick, gyroPort);  % +1 or -1

    %=== 2) Track straight using gyro ===
    startAng = double(brick.GyroAngle(gyroPort));
    prevErr = 0;
    worseCount = 0; flipped = false;

    function forwardMove(brick, ultrasoundPort, wallLength, gyroPort)
        ang = double(brick.GyroAngle(gyroPort));
        err = gyroSign * (ang - startAng);   % +err = drifting LEFT

        % deadband
        if abs(err) < eps
            err = 0;
        end

        % PD correction
        dErr = (err - prevErr) / dt;
        corr = Kp*err + Kd*dErr;

        % Apply steering: if err>0 (left drift) -> push right harder, ease left
        leftCmd  = baseLeft  + (+corr);
        rightCmd = baseRight - (+corr);

        % clamp to EV3 limits
        leftCmd  = max(-100, min(100, leftCmd));
        rightCmd = max(-100, min(100, rightCmd));

        brick.MoveMotor('A', leftCmd);
        brick.MoveMotor('B', rightCmd);

        % stop if too close to wall
        d = brick.UltrasonicDist(ultrasoundPort);
        if d < (wallLength/2) - 5
            return;
        end

        %=== 3) Safety: if error keeps getting worse, flip sign once ===
        if abs(err) > abs(prevErr) + 0.2
            worseCount = worseCount + 1;
        else
            worseCount = max(0, worseCount - 1);
        end
        if (~flipped) && worseCount >= 8
            gyroSign = -gyroSign;     % you had the wrong sign; fix on the fly
            worseCount = 0; flipped = true;
        end

        prevErr = err;
        pause(dt);
    end

    t0 = tic;
    % FIX : check if ultra < 255, and go until ultra - wall length
    frontDistance = brick.UltrasonicDist(ultrasoundPort);
    target = frontDistance - wallLength;
    
    if frontDistance < 255
        currDistance = frontDistance;
        while currDistance > target
            forwardMove(brick, ultrasoundPort, wallLength, gyroPort)
            currDistance = brick.UltrasonicDist(ultrasoundPort);
        end
    else
        while toc(t0) < distance
            forwardMove(brick, ultrasoundPort, wallLength, gyroPort)
        end
    end


    brick.StopAllMotors('Brake');
end

function s = calibrateGyroSign(brick, gyroPort)
    % Pulse a small LEFT turn; see which way the gyro angle moves.
    a0 = double(brick.GyroAngle(gyroPort));
    % left pulse: left motor backward, right motor forward (spin in place)
    brick.MoveMotor('A', -20); brick.MoveMotor('B', 20);
    pause(0.25);
    brick.StopAllMotors('Brake'); pause(0.1);
    a1 = double(brick.GyroAngle(gyroPort));
    s = sign(a1 - a0);     % +1 if left turn raises angle, -1 if it lowers
    if s == 0, s = 1; end  % fallback

    % afterward adjust
    brick.MoveMotor('A', 20); brick.MoveMotor('B', -20);
    pause(0.25);
    brick.StopAllMotors('Brake'); pause(0.1);
end

function wallArray = radar(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength)
    % in centimeters
    buffer = 5;

    distances = [];  % initialize empty list
    
    distance = brick.UltrasonicDist(ultrasoundPort);
    distances(end+1) = distance;  % append to list

    brick.MoveMotorAngleRel('C', 20, 0, 'Brake');
    pause(1);

    for i = 0:2
        brick.MoveMotorAngleRel('C', 20, 90, 'Brake');
        brick.WaitForMotor('C');
        distance = brick.UltrasonicDist(ultrasoundPort);
        distances(end+1) = distance;  % append to list
    end

    display(distances);
    
    brick.MoveMotorAngleRel('C', 20, -270, 'Brake');
    brick.WaitForMotor('C');


    angle = brick.GyroAngle(gyroPort);   % moving right is negative
    disp(angle);
    
    % mock sensor readings (1=north, 2=east, 3=south, 4=west)
    north = distances(1);
    east  = distances(2);
    south = distances(3);
    west  = distances(4);
    
    % --- compute how many -90° steps we've taken (with ±5° tolerance) ---
    % Treat right turns (negative angle) as positive step-progress.
    ang = mod(-angle, 360);              % 0..360, increases as you turn right
    k = floor((ang + 5) / 90);           % count 90° steps once past 85°, 175°, 265°, 355°
    k = mod(k, 4);                       % only need 0..3 rotations
    

    %{
    % --- rotate readings k times so that old 4 -> new 1, others shift up ---
    if k == 0
        distances_rot = distances;
    elseif k == 1
        distances_rot = [distances(4) distances(1) distances(2) distances(3)];
    elseif k == 2
        distances_rot = [distances(3) distances(4) distances(1) distances(2)];
    else % k == 3
        distances_rot = [distances(2) distances(3) distances(4) distances(1)];
    end
    
    % unpack (optional)
    north = distances_rot(1);
    east  = distances_rot(2);
    south = distances_rot(3);
    west  = distances_rot(4);
    %}


    % init wall states
    n=0; e=0; s=0; w=0;

    % nested update function
    function wallArray = updateWallArray(wallArray, currentGrid, n, e, s, w)
        row = currentGrid(1);
        col = currentGrid(2);
        wallArray(row, col, :) = logical([n, e, s, w]);
    end

    % north/south detection
    northside = north + south;
    if (northside > wallLength - buffer && northside < wallLength + buffer)
        n=1; s=1;
    elseif (north < south)
        if (north < wallLength)
            n=1;
        end
    else
        if (south < wallLength)
            s=1;
        end
    end

    % east/west detection
    eastside = east + west;
    if (eastside > wallLength - buffer && eastside < wallLength + buffer)
        e=1; w=1;
    elseif (east < west)
        if (east < wallLength)
            e=1;
        end
    else
        if (west < wallLength)
            w=1;
        end
    end

    % update the array
    wallArray = updateWallArray(wallArray, currentGrid, n, e, s, w);

    % print result
    disp('Wall array slice at current grid:');
    disp(squeeze(wallArray(currentGrid(1), currentGrid(2), :))');
end

function manual(brick)
    brick.beep(); 
    
    global key
    InitKeyboard();
    while 1
        pause(0.1);
        switch key
            case'uparrow'
                brick.MoveMotor('AB', -50);
            case'downarrow'
                brick.MoveMotor('AB', 50);
            case'leftarrow'
                brick.MoveMotorAngleRel('A', 20, 90, 'Brake');
                brick.MoveMotorAngleRel('B', 20, -90, 'Brake');
            case'rightarrow'
                brick.MoveMotorAngleRel('A', 20, -90, 'Brake');
                brick.MoveMotorAngleRel('B', 20, 90, 'Brake');
            case'w'
                brick.MoveMotorAngleRel('D', 50, 500, 'Brake');
                pause(2.5);
            case'e'
                brick.MoveMotorAngleRel('D', 50, -500, 'Brake');
                pause(2.5);
            case 'q'
                brick.StopMotor('AB', 'Brake');
            case 'l'
                break;
        end
    end
    CloseKeyboard();
end


function colorName = detectColor(brick, sensorPort)
    % Set color sensor to RGB mode (Mode = 4 as per the library)  
    brick.SetColorMode(sensorPort, 4);  % RGB mode :contentReference[oaicite:1]{index=1}  
    pause(0.1);  % allow sensor to update
    
    rgb = brick.ColorRGB(sensorPort);
    R = double(rgb(1));
    G = double(rgb(2));
    B = double(rgb(3));

    display(R)
    display(G)
    display(B)
    
    if R > 50 && G > 50
        colorName = 'yellow';
    elseif G > B && G > 50
        colorName = 'green';
    elseif B > 50
        colorName = 'blue';
    elseif R > 50
        colorName = 'red';
    else
        colorName = 'other';
    end
end
%wallArray = radar(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength);

%forward(brick, ultrasoundPort, 100, wallLength, gyroPort);

%main(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength, hasGreen);

%left(brick, gyroPort);

%radar(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength)

cb = true;
cg = true;
while true
    brick.MoveMotor('AB', -50);
    
    color = detectColor(brick, 3);

    if color == "red"
        brick.StopAllMotors();
        pause(1);

    elseif color == "blue" && cb
        brick.StopAllMotors();
        brick.beep();
        pause(.1);
        brick.beep();
        pause(.1);

        cb = false;
    elseif color == "green" && cg
        brick.StopAllMotors();
        brick.beep();
        pause(.1);
        brick.beep();
        pause(.1);
        brick.beep();
        pause(.1);

        cg = false;
    end

    pause(.05);
end

StopAllMotors(brick);
DisconnectBrick(brick);