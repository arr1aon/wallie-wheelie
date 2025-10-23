brick = ConnectBrick("OOO");

ultrasoundPort = 1;
gyroPort = 2;

currentGrid = [1, 1];
wallArray = false(6, 3, 4);

brick.GyroCalibrate(gyroPort);

wallLength = 58.42;

angle = NaN;
t0 = tic;
while isnan(angle) && toc(t0) < 5   % poll up to 5s
    pause(0.05);
    angle = brick.GyroAngle(gyroPort);
end

brick.beep();

function main(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength)
    wallArray = radar(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength);
    
    % Extract the list of wall values for the current grid
    wallList = squeeze(wallArray(currentGrid(1), currentGrid(2), :))';
    
    % Find zero positions
    zeroPositions = find(wallList == 0);
    if ~isempty(zeroPositions)
        randIndex = zeroPositions(randi(numel(zeroPositions))); % Pick random one
        fprintf("i found a path :D\n");
    else
        randIndex = [];  % No zeros found
        fprintf("im stuck ;(\n");
    end

    display(randIndex)

    if randIndex == 1
        % facing the right direction
    
    elseif randIndex == 2
        right(brick, gyroPort);
    
    elseif randIndex == 3
        % Randomly pick right or left, and run it twice
        if randi(2) == 1
            right(brick, gyroPort);
            right(brick, gyroPort);
        else
            left(brick, gyroPort);
            left(brick, gyroPort);
        end
    
    elseif randIndex == 4
        left(brick, gyroPort);
    end

    forward(brick, ultrasoundPort, 2, wallLength);

    reAdjust(brick, gyroPort);

    main(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength);
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

function forward(brick, ultrasoundPort, distance, wallLength)
    current = brick.UltrasonicDist(ultrasoundPort);
    target = current - distance;

    time = 0;

    %display(target);

    while time < distance
        brick.MoveMotor('A', -50);
        brick.MoveMotor('B', -60);
        pause(.05);
        time = time + 0.05;

        %display(current);
        current = brick.UltrasonicDist(ultrasoundPort);

        if current < wallLength/2 - 10
            break
        end
    end

    brick.StopAllMotors('Brake');
end

function back(brick)
    brick.MoveMotor('AB', 50);
    pause(1);
    brick.StopAllMotors('Brake');
end

function wallArray = radar(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength)
    % in centimeters
    buffer = 5;

    distances = [];  % initialize empty list
    
    distance = brick.UltrasonicDist(ultrasoundPort);
    distances(end+1) = distance;  % append to list
    
    for i = 0:2
        brick.MoveMotorAngleRel('C', 20, 90, 'Brake');
        pause(1);
        distance = brick.UltrasonicDist(ultrasoundPort);
        distances(end+1) = distance;  % append to list
    end
    
    brick.MoveMotorAngleRel('C', 20, -270, 'Brake');
    pause(1);


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
    %disp('Wall array slice at current grid:');
    %disp(squeeze(wallArray(currentGrid(1), currentGrid(2), :))');
end

%wallArray = radar(brick, ultrasoundPort, gyroPort, wallArray, currentGrid);

%forward(brick, ultrasoundPort, 5, wallLength);

main(brick, ultrasoundPort, gyroPort, wallArray, currentGrid, wallLength);

%left(brick, gyroPort);

DisconnectBrick(brick);