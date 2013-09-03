function setpath

%__________________________________________________
% Setpath: Sets the path for the matlab functions
%___________________________________________________

if ispc,        PATH = genpath([pwd '\']); %PATHData = genpath([pwd '\..\data\']);
elseif isunix,  PATH = genpath([pwd '/']); %PATHData = genpath([pwd '/../data/']);
else error(' Operative system not recognized.');
end;

path(path,[PATH]);
%path(path,[PATHData]);

return 