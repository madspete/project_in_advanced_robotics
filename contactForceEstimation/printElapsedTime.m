function printElapsedTime(timeStartHour,timeStartMin,timeStartSec)

    time = clock;
    timeDisplay = num2str((time(4)-timeStartHour)*3600 + (time(5)-timeStartMin)*60 + floor(time(6)-timeStartSec));
    disp(strcat('Program finished executing in:', timeDisplay, ' seconds'))
end