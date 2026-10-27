function runWithLog(scriptName, logName)
%RUNWITHLOG Run a project script and save everything it prints to results/<logName>.
%   runWithLog('run_step3_trajectory_tracking', 'step3_log.txt')
%
%   The text is captured with evalc and written to the log file after the script
%   finishes (MATLAB's diary command does not record output when MATLAB is driven
%   through the MCP server, which left earlier logs empty). If the script fails,
%   everything printed before the error is still saved, the error message is added at the
%   end of the log, and the error is then re-thrown.

    arguments
        scriptName (1, :) char
        logName (1, :) char
    end

    scriptDir   = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(scriptDir);
    scriptPath  = fullfile(scriptDir, [scriptName '.m']);
    logPath     = fullfile(projectRoot, 'results', logName);
    if ~isfile(scriptPath)
        error('runWithLog:noScript', 'Script not found: %s', scriptPath);
    end
    if ~isfolder(fileparts(logPath)), mkdir(fileparts(logPath)); end

    [output, failure] = captureScriptOutput(scriptPath);

    fid = fopen(logPath, 'w');
    fwrite(fid, regexprep(output, '</?strong>', ''));   % drop bold markup from table display
    fclose(fid);
    fprintf('%s', output);
    fprintf('Log saved to %s\n', logPath);
    if ~isempty(failure)
        rethrow(failure);
    end
end

function [output, failure] = captureScriptOutput(scriptPath) %#ok<INUSD>   % scriptPath is used inside the evalc string
%CAPTURESCRIPTOUTPUT Run a script with evalc in its own function workspace.
%   Project scripts start with 'clear', which wipes the workspace they run in.
%   Running them from this separate function keeps the caller's variables safe.
%   The try/catch is inside the evalc string so that the text printed before an error is kept.
    output = evalc('try, run(scriptPath); catch scriptError, fprintf(''SCRIPT FAILED: %s\n'', scriptError.message); end');
    % The script's own 'clear' also removes any variable defined before it ran
    if exist('scriptError', 'var')
        failure = scriptError;
    else
        failure = [];
    end
end
