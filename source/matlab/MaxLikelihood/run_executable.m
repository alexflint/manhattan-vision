function [ varargout ] = run_executable( cmd, working_dir )
%RUN_EXECUTABLE Summary of this function goes here
%   Detailed explanation goes here

% Execute command
if (nargin >= 2)
    cd(working_dir);
end
[status,output]=system(cmd);
if (status ~= 0)
    error(['Error in executable: ' output]);
end

% For indoor_context binaries
setenv('LD_PRELOAD', '/usr/lib/liblapack.so');

% Split lines and parse results
lines = strsplit(output, char(10));  % char(10) is newline, '\n' fails
for i = 1:nargout
    line = lines{length(lines)-i};
    varargout(nargout+1-i) = {str2num(line)}; %#ok<AGROW,ST2NM>
end

end
