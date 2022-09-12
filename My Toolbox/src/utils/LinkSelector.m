classdef BodySelector < handle
    %  BodySelector opens a DDG dialog that lets the user select
    %  from a list of Bodies. Once the user accepts the changes
    %  (or cancels the dialog), a callback is invoked with the closure
    %  action and selected body.
    %
    %  Sample use:
    %   selector = BodySelector;
    %   % The first argument is the message type to select by default
    %   selector.openDialog('body0', @(isAccepted,RigidBody), disp(RigidBody));

    properties (SetAccess = private)
        bodyName = ''
        bodyList = {}
        CloseFcnHandle = function_handle.empty
    end

    methods

        function obj = BodySelectors()
        end

        function dlg = openDialog(obj, defaultSelection, spaceRobot, closeFcnHandle)
            % closeFcnHandle: handle to function that takes two arguments
            %   closeFcn(isAcceptedSelection, RigidBody)
            %      isAcceptedSelection: true if user clicked on 'ok', false
            %        if user clicked on 'cancel' or closed window
            %      bodyName: last selected body in list (string)

            assert(ischar(defaultSelection) || isempty(defaultSelection));
            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;

            obj.bodyList = [spaceRobot.BaseName; spaceRobot.BodyNames'];

            dlg = DAStudio.Dialog(obj);

            if isempty(defaultSelection)
                return;
            end

            % Find initial rigid body, if any
            index = find(strcmpi(defaultSelection, obj.bodyList));

            if isempty(index)
                index = 1; %If no matching value is found, use first item in list
            end

            dlg.setWidgetValue('bodylist', index - 1); % zero-based
            obj.bodyName = obj.bodyList{index};
        end

    end

    methods (Hidden)
        % Called when user selects an item from the list
        function dlgCallback(obj, dlg, tag, value) %#ok<INUSL>
            obj.bodyName = obj.bodyList{value + 1}; % value is zero-based
            dlg.refresh;
        end

        function dlgClose(obj, closeaction)
            % closeaction is 'ok' if user clicked OK
            %                'cancel' if user clicked cancel or closed window
            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');

                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.bodyName);
                catch
                    % Absorb all errors. If they are propagated back to
                    % DDG, this causes MATLAB to crash, (Can't convert to
                    % warnings are not as they are not displayed either).
                end

            end

        end

        function dlgstruct = getDialogSchema(obj)
            bodylist.Name = '';
            bodylist.Type = 'listbox';
            bodylist.Entries = obj.bodyList;
            bodylist.Tag = 'bodylist';
            bodylist.MultiSelect = false;
            bodylist.ObjectMethod = 'dlgCallback'; % call method on UDD source object
            bodylist.MethodArgs = {'%dialog', '%tag', '%value'}; % object handle is implicit first arg
            bodylist.ArgDataTypes = {'handle', 'string', 'mxArray'}; % 'handle' is type of %dialog
            bodylist.Value = 0;
            bodylist.NameLocation = 2; % top left

            % Main dialog
            dlgstruct.DialogTitle = 'Select Body';
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
            dlgstruct.StandaloneButtonSet = ...
            {'Ok', 'Cancel'}; % also available: 'Help', 'Revert', 'Apply'

            dlgstruct.Items = {bodylist};
        end

    end

end
