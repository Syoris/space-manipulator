function Struct2File(sr_info, savePath)
%Struct2File Export struct to .m file
%
% Input:
%   - sr_info   Structure to write
%
%   - savePath  Where to save file

if nargin<2
    savePath = '';
end

fileName = [sr_info.Name, '_info_struct.m'];
fullPath = fullfile(savePath, fileName);

fid = fopen(fullPath,'w');

%% General Infos
fprintf(fid, 'function sr_info = %s()\n', fileName(1:end-2));

fprintf(fid, '%%%% General Infos\n');
fprintf(fid, '\tsr_info = struct();\n');
fprintf(fid, '\tsr_info.Name = ''%s'';\n', sr_info.Name);
fprintf(fid, '\tsr_info.nk = %i;\n', sr_info.nk);
fprintf(fid, '\tsr_info.n = %i;\n', sr_info.n);
fprintf(fid, '\tsr_info.N = %i;\n', sr_info.N);

fprintf(fid, '\n\tsr_info.A = cell(1, 2);\n');
fprintf(fid, '\tsr_info.M = cell(1, 2);\n');
fprintf(fid, '\tsr_info.P = cell(1, 2);\n');
    
fprintf(fid, ['\tsr_info.jnt_idx = [' array2str(sr_info.jnt_idx, 'i'), '];\n'], sr_info.jnt_idx.'); % write it
fprintf(fid, '\tsr_info.RFunc = ''%s'';\n', sr_info.RFunc);
fprintf(fid, ['\tsr_info.BodyNames = {' repmat('''%s'' ', 1, sr_info.nk), '};\n'], sr_info.BodyNames{:}); % write it

%% A Mats
fprintf(fid, '\n %%%% A\n');
fprintf(fid, '\tsr_info.A{1} = [...\n');
fprintf(fid, ['\t\t', array2str(sr_info.A{1}, 'f'), '\n'], sr_info.A{1}.');
fprintf(fid, '\t];\n');

fprintf(fid, '\n\tsr_info.A{2} = zeros(6, 6, sr_info.nk);\n'); % sr_info.A{2} = zeros(6, 6, sr_info.nk);
for i=1:sr_info.nk
    fprintf(fid, '\tsr_info.A{2}(:, :, %i) = [...\n', i); % sr_info.A{2}(:, :, 1) = [...  
    fprintf(fid, ['\t\t', array2str(sr_info.A{2}(:, :, i), 'f'), '\n'], sr_info.A{2}(:, :, i).'); % write it
    fprintf(fid, '\t];\n');
end

%% M
fprintf(fid, '\n %%%% M\n');
fprintf(fid, '\tsr_info.M{1} = [...\n');
fprintf(fid, ['\t\t', array2str(sr_info.M{1}, 'f'), '\n'], sr_info.M{1}.');
fprintf(fid, '\t];\n');

fprintf(fid, '\n\tsr_info.M{2} = zeros(6, 6, sr_info.nk);\n'); % sr_info.M{2} = zeros(6, 6, sr_info.nk);
for i=1:sr_info.nk
    fprintf(fid, '\tsr_info.M{2}(:, :, %i) = [...\n', i); % sr_info.M{2}(:, :, 1) = [...  
    fprintf(fid, ['\t\t', array2str(sr_info.M{2}(:, :, i), 'f'), '\n'], sr_info.M{2}(:, :, i).'); % write it
    fprintf(fid, '\t];\n');
end

%% P
fprintf(fid, '\n %%%% P\n');
fprintf(fid, '\tsr_info.P{1} = [...\n');
fprintf(fid, ['\t\t', array2str(sr_info.P{1}, 'f'), '\n'], sr_info.P{1}.');
fprintf(fid, '\t];\n');

fprintf(fid, '\n\tsr_info.P{2} = zeros(6, 1, sr_info.nk);\n'); % sr_info.P{2} = zeros(6, 6, sr_info.nk);
for i=1:sr_info.nk
    fprintf(fid, '\tsr_info.P{2}(:, :, %i) = [...\n', i); % sr_info.P{2}(:, :, 1) = [...  
    fprintf(fid, ['\t\t', array2str(sr_info.P{2}(:, :, i), 'f'), '\n'], sr_info.P{2}(:, :, i).'); % write it
    fprintf(fid, '\t];\n');
end

%% Close file
fprintf(fid, 'end');
fclose(fid);

    function arrayStr = array2str(inputArray, inputType)
        [mrows, ncols] = size(inputArray);
        outputstr = ['%' num2str(mrows) inputType ' ']; % template for the string, you put your datatype here
        outputstr = repmat(outputstr, 1, ncols); % replicate it to match the number of columns
        arrayStr = outputstr;
    end
end