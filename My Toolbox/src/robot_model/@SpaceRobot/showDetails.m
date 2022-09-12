%TODO: Show joint config var
function showDetails(obj)
    %showdetails Display details of the robot
    %   showdetails(ROBOT) displays details of each body in the
    %   robot including the body name, associated joint name and
    %   type, and its parent name and children names.
    %
    %   Example:
    %       % Display details of myRobot robot
    %       showdetails(myRobot);
    %
    %   See also show
    fprintf('--------------------\n');
    fprintf('%s: (%d bodies)\n\n', obj.Name, int32(obj.NumBodies));

    widMaxBodyName = 10;
    widMaxJointName = 10;

    for i = 1:obj.NumBodies
        widMaxBodyName = ...
            max(widMaxBodyName, length(obj.Bodies{i}.Name) + 5);
        widMaxJointName = ...
            max(widMaxJointName, length(obj.Bodies{i}.Joint.Name) + 5);
    end

    fprintf('%4s    %*s   %*s   %*s   %*s  %*s(Idx)   Children Name(s)\n', ...
        'Idx', ...
        widMaxBodyName, 'Body Name', ...
        widMaxJointName, 'Joint Name', ...
        widMaxJointName, 'Joint Type', ...
        12, 'ConfigSymb', ...
        widMaxBodyName + 2, 'Parent Name');

    fprintf('%4s    %*s   %*s   %*s   %*s  %*s-----   ----------------\n', ...
        '---', ...
        widMaxBodyName, '---------', ...
        widMaxJointName, '----------', ...
        widMaxJointName, '----------', ...
        12, '----------', ...
        widMaxBodyName + 2, '-----------');

    for i = 1:obj.NumBodies

        jointname = obj.Bodies{i}.Joint.Name;
        jointtype = obj.Bodies{i}.Joint.Type;
        bodyName = obj.Bodies{i}.Name;
        bodyVar = obj.Bodies{i}.Joint.SymbVar;

        fprintf('%4d', i);
        fprintf('   %*s', widMaxBodyName, bodyName);
        fprintf('   %*s', widMaxJointName, jointname);
        fprintf('   %*s', widMaxJointName, jointtype);
        fprintf(' %*s', 12, bodyVar);

        pid = obj.Bodies{i}.ParentId;

        if pid > 0
            parent = obj.Bodies{pid};
        else
            parent = obj.Base;
        end

        pid = obj.Bodies{i}.ParentId;

        % estimate the number of digits for a body index
        p = pid;
        widID = 0;

        while p > 0.2
            p = floor(p / 10);
            widID = widID + 1;
        end

        widID = max(1, widID);

        fprintf('   %*s(%*d)   ', widMaxBodyName + 8 - widID, parent.Name, widID, int32(pid));

        childrenList = obj.Bodies{i}.Children;

        for j = 1:length(childrenList)
            childrenName = childrenList{j}.Name;
            childrenId = childrenList{j}.Id;
            fprintf('%s(%d)  ', childrenName, int32(childrenId));
        end

        fprintf('\n');
    end

    fprintf('--------------------\n');
end
