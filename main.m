brick = ConnectBrick("OOO");

currentGrid = [1, 1];

wallArray = false(6, 3, 4);

function main()
    radar();
end

function turn(left)
    if left
        
    else

    end
end

function forward()

end

function wallArray = radar(wallArray, currentGrid)
    % in centimeters
    wallLength = 58.42;
    buffer = 5;

    % mock sensor readings
    north = 83.5;
    east  = 28.2;
    south = 32.1;
    west  = 25.9;

    % wall states
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
    elseif (north > south)
        if (north < wallLength - buffer)
            n=1;
        end
    else
        if (south < wallLength - buffer)
            s=1;
        end
    end

    % east/west detection
    eastside = east + west;
    if (eastside > wallLength - buffer && eastside < wallLength + buffer)
        e=1; w=1;
    elseif (east > west)
        if (east < wallLength - buffer)
            e=1;
        end
    else
        if (west < wallLength - buffer)
            w=1;
        end
    end

    % update the array
    wallArray = updateWallArray(wallArray, currentGrid, n, e, s, w);

    % print result
    disp('Wall array slice at current grid:');
    disp(squeeze(wallArray(currentGrid(1), currentGrid(2), :))');
end

wallArray = radar(wallArray, currentGrid);
%main();

brick.delete();
delete(brick);
clear brick;