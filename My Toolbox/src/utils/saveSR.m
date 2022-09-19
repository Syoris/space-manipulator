function saveSR(sr, varargin)
    %save Save SR to .mat file
    %
    % SAVE(SR, ___) saves under SR_NAME.mat.
    %
    % SAVE(___, Name, Value)
    %      'FileName'       - Save file name
    %
    %      'Path'           - Path where to save file

    parser = inputParser;
    parser.addParameter('FileName', sr.Name);
    parser.addParameter('Path', '');

    parser.parse(varargin{:});

    fileName = parser.Results.FileName;
    savePath = parser.Results.Path;

    if ~isempty(savePath)
        saveName = strcat(savePath, fileName);
    else
        saveName = fileName;
    end
    
    saveName = strcat(saveName, '.mat');

    msg = sprintf('Saving to: %s\n', saveName);
    fprintf(msg);

    save(saveName, 'sr');

end
