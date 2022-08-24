classdef LinkSelector < handle 
    %  LinkSelector opens a DDG dialog that lets the user select
    %  from a list of Links. Once the user accepts the changes
    %  (or cancels the dialog), a callback is invoked with the closure
    %  action and selected link. 
    % 
    %  Sample use:
    %   selector = LinkSelector;
    %   % The first argument is the message type to select by default    
    %   selector.openDialog('body0', @(isAccepted,RigidBody), disp(RigidBody));
    
    properties(SetAccess=private)
        linkName = ''
        linkList = {}
        CloseFcnHandle = function_handle.empty
    end
    
    
    methods       
        function obj = LinkSelectors()
        end
                
        function dlg = openDialog(obj, defaultSelection, spaceRobot, closeFcnHandle)
            % closeFcnHandle: handle to function that takes two arguments
            %   closeFcn(isAcceptedSelection, RigidBody)
            %      isAcceptedSelection: true if user clicked on 'ok', false
            %        if user clicked on 'cancel' or closed window
            %      linkName: last selected link in list (string)        
            
            assert(ischar(defaultSelection) || isempty(defaultSelection));
            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;
            
            obj.linkList = [spaceRobot.BaseName; spaceRobot.LinkNames'];
            
            dlg = DAStudio.Dialog(obj);            
            if isempty(defaultSelection)
                return;
            end
            
            % Find initial rigid body, if any
            index = find(strcmpi(defaultSelection, obj.linkList));
            if isempty(index)
                index = 1; %If no matching value is found, use first item in list
            end
            dlg.setWidgetValue('linklist', index-1); % zero-based
            obj.linkName = obj.linkList{index};
        end
    end
    
    
    methods(Hidden)
        % Called when user selects an item from the list
        function dlgCallback(obj, dlg, tag, value) %#ok<INUSL>
            obj.linkName = obj.linkList{value+1}; % value is zero-based
            dlg.refresh;
        end
        
        
        function dlgClose(obj, closeaction)
            % closeaction is 'ok' if user clicked OK
            %                'cancel' if user clicked cancel or closed window
             if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.linkName);
                catch 
                    % Absorb all errors. If they are propagated back to 
                    % DDG, this causes MATLAB to crash, (Can't convert to 
                    % warnings are not as they are not displayed either).
                end
             end
        end
        
        
        function dlgstruct = getDialogSchema(obj)
            linklist.Name    = '';
            linklist.Type    = 'listbox';
            linklist.Entries = obj.linkList;
            linklist.Tag     = 'linklist';
            linklist.MultiSelect = false;
            linklist.ObjectMethod = 'dlgCallback'; % call method on UDD source object
            linklist.MethodArgs = {'%dialog', '%tag', '%value'}; % object handle is implicit first arg
            linklist.ArgDataTypes = {'handle', 'string', 'mxArray'}; % 'handle' is type of %dialog
            linklist.Value = 0;
            linklist.NameLocation = 2; % top left
                        
            % Main dialog
            dlgstruct.DialogTitle = 'Select Link';
%             dlgstruct.HelpMethod = 'robotics.slros.internal.helpview';
%             dlgstruct.HelpArgs =  {'rstGetTransform'};
            dlgstruct.CloseMethod = 'dlgClose';            
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};
            
            % Make this dialog modal wrt to other DDG dialogs 
            % (doesn't block MATLAB command line)
            dlgstruct.Sticky = true; 
            
            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            dlgstruct.StandaloneButtonSet =  ...
                {'Ok', 'Cancel'}; % also available: 'Help', 'Revert', 'Apply'
            
            dlgstruct.Items = {linklist};
        end
    end
end

