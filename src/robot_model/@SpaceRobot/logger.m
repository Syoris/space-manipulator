function logger(obj, msg, level)
%logger Print msg to console based if level is higher than SpaceRobot.Logging
% msg: Message to print
% level: Logging level of the message. Can be: 'error', 'warning', 'info', 'debug'
%
% Print as: [SpaceRobot Name] - [fname] - [msg]


    % Check level is ok
    level = validatestring(level, obj.LogLevels, mfilename, 'level');

    % Find caller name
    fStack = dbstack();
    if size(fStack, 1) > 1
        fName = fStack(2).name;
    else
        fName = '';
    end
    
    msgStr = sprintf('[%s] %s - %s - %s\n', obj.Name, ['(', level, ')'], fName, msg);

    switch level
        case 'error'
            error(msgStr);
        
        case 'warning'
            if ~strcmp(obj.Logging, 'error')
                warning(msgStr);
            end 
        
        case 'info'
            if strcmp(obj.Logging, 'info') || strcmp(obj.Logging, 'debug')
                fprintf(msgStr);
            end 


        case 'debug'
            if strcmp(obj.Logging, 'debug')
                fprintf(msgStr);
            end 
    end
end