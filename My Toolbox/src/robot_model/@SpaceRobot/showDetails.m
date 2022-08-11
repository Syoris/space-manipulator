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
    fprintf('%s: (%d links)\n\n', obj.Name, int32(obj.NumLinks));
    
    widMaxBodyName = 10;
    widMaxJointName = 10;
    
    for i = 1:obj.NumLinks
        widMaxBodyName = ...
            max(widMaxBodyName, length(obj.Links{i}.Name)+5);
        widMaxJointName = ...
            max(widMaxJointName, length(obj.Links{i}.Joint.Name)+5);
    end
    
    fprintf('%4s    %*s   %*s   %*s   %*s  %*s(Idx)   Children Name(s)\n', ...
        'Idx', ...
        widMaxBodyName, 'Link Name',...
        widMaxJointName, 'Joint Name',...
        widMaxJointName, 'Joint Type',...
        12, 'ConfigSymb', ...
        widMaxBodyName+2, 'Parent Name' );
    
    fprintf('%4s    %*s   %*s   %*s   %*s  %*s-----   ----------------\n', ...
        '---', ...
        widMaxBodyName, '---------',...
        widMaxJointName, '----------',...
        widMaxJointName, '----------',...
        12, '----------',...
        widMaxBodyName+2, '-----------');
    
    for i = 1:obj.NumLinks
        
        jointname = obj.Links{i}.Joint.Name;
        jointtype = obj.Links{i}.Joint.Type;
        linkName = obj.Links{i}.Name;
        linkVar = obj.Links{i}.Joint.SymbVar;
        
        fprintf('%4d', i);
        fprintf('   %*s', widMaxBodyName, linkName);
        fprintf('   %*s', widMaxJointName, jointname);
        fprintf('   %*s', widMaxJointName, jointtype);
        fprintf(' %*s', 12, linkVar);
        
        
        pid = obj.Links{i}.ParentId;
        if pid > 0
            parent = obj.Links{pid};
        else
            parent = obj.Base;
        end
        pid = obj.Links{i}.ParentId;
        
        % estimate the number of digits for a body index
        p = pid;
        widID = 0;
        while p > 0.2
            p = floor(p/10);
            widID = widID+1;
        end
        widID = max(1, widID);
        
        fprintf('   %*s(%*d)   ', widMaxBodyName+8-widID, parent.Name, widID, int32(pid));
        
        childrenList = obj.Links{i}.Children;
        for j = 1:length(childrenList)
            childrenName = childrenList{j}.Name;
            childrenId = childrenList{j}.Id;
            fprintf('%s(%d)  ', childrenName, int32(childrenId) );
        end
        
        fprintf('\n');
    end
    
    fprintf('--------------------\n');
end