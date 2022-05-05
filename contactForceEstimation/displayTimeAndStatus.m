function displayTimeAndStatus(statusMessage,timeStartHour,timeStartMin,timeStartSec,timeSinceStartInSeconds)

    timeStartSec = floor(timeStartSec);

    timeSec = floor(mod(timeSinceStartInSeconds,60));
    timeMin = floor(mod(timeSinceStartInSeconds/60,60));
    timeHour = floor(floor(timeSinceStartInSeconds/3600));
    if timeStartSec + timeSec > 60
        timeMin = timeMin + 1;
        timeSec = timeStartSec + timeSec - 60;
    else
        timeSec = timeStartSec + timeSec;
    end
    if timeStartMin + timeMin > 60
        timeHour = timeHour + 1;
        timeMin = timeStartMin + timeMin - 60;
    else
        timeMin = timeStartMin + timeMin;
    end
    if timeStartHour + timeHour < 24
        timeHour = timeStartHour + timeHour; 
    else
        timeHour = 0;
    end

    % Convect to strings, add leading zeros. Add leading zeros
    secDisplay = toDisplayTime(timeSec);
    minDisplay = toDisplayTime(timeMin);
    hourDisplay = toDisplayTime(timeHour);

    disp(strcat(statusMessage, ' at time:', hourDisplay, ':', minDisplay, ':', secDisplay))
end

function str = toDisplayTime(time)
    if time < 10
        str = strcat('0',num2str(time));
    else
        str = num2str(time);
    end
end