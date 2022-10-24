classdef MaskUtils < handle
    % Class to update block masks

    methods(Static)
        function updateBodyList(block, parameterName)
            %updateMaskBodyList Update the list of available Bodies
            %   When the algorithm block dialog is opened, for dialogs that
            %   allow users to specify bodies (e.g. Get Jacobian and Get
            %   Transform blocks), this function is used to populate the
            %   list of source and target bodies. The parameters to update
            %   are passed as character array inputs to this callback.

            %Do nothing if called in a library
            sys = bdroot(block);
            if bdIsLibrary(sys)
                return
            end
            
            %Do nothing if the model is running
            if MaskUtils.checkIfRunning(sys)
                return
            end
            
            %Use slResolve to convert the RigidBodyTree parameter to an
            %actual value that can be used in the mask. slResolve evaluates
            %a SL starting at the stated level (here: block level).
            maskValue = get_param(block, 'spaceRobot');
            spaceRobot = slResolve(maskValue, block);

            if isempty(spaceRobot)
                %If slresolve fails to resolve, it will return empty
                error(message('Invalid SpaceRobot', maskValue));
            end
            
            %Get the currently selected value
            currentBody = get_param(block, parameterName);
            
            %Create a dialog object and populate it
            bodyDlg = BodySelector();
            bodyDlg.openDialog(currentBody, spaceRobot, @dialogCloseCallback);
            
            %Dialog close callback: set associated parameter value
            function dialogCloseCallback(isAcceptedSelection, selectedBody)
                if isAcceptedSelection
                    set_param(block, parameterName, selectedBody);
                end
            end
        
        
        
        end
    end

    %% Utils
    methods(Static)
        function TF = checkIfRunning(sys)
            %checkIfRunning Return TRUE the model is running            
            TF = false;
            
            invalidStatus = {'external','running','compiled','restarting','paused','terminating'};
            if any(strcmpi(get_param(sys,'SimulationStatus'),invalidStatus))
                TF = true;
            end
        end
    end
end