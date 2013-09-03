function [ S ] = loadGroundTruth( filename )
%LOADGROUNDTRUTH Summary of this function goes here
%  Detailed explanation goes here

file = fopen( filename );

str = fgetl( file );
num = sscanf( str, 'numOfFrames: %d' );
str = fgetl(file);

str = fgetl(file);
cam_container.position = [];
cam_container.orientation = [];
for i=1:num
    str = fgetl(file);
    num = sscanf( str, '%d, (%f, %f, %f), (%f, %f, %f, %f)' );
    cam_container.position(:,end+1) = num(2:4);
    cam_container.orientation(:,end+1) = num(5:8);
end

str = fgetl(file);
static_container = [];
moving_container = [];

while 1
    str = fgetl(file);
    if ~ischar(str), break, end
    ret = sscanf( str, 'Point %d: %s' );
    
    if [ char(ret(2:end))' ] == 'Static'
        str = fgetl(file);
        num = sscanf( str, 'Position: (%f, %f, %f)' );
        static_container(end+1).position = num;
%         static.position = num;
        
        str = fgetl(file);
        
        str = fgetl(file);
        static_container(end).frame = [];
        static_container(end).projection = [];
%         static.frame = [];
%         static.projection = [];
        while ~isempty(str) && ischar(str)
            num = sscanf( str, '%d, (%f, %f)' );
            static_container(end).frame(end+1) = num(1);
            static_container(end).projection(:,end+1) = num(2:3);
            str = fgetl(file);
        end
		static_container(end).pointIndex = ret(1) ;
%         static_container(end+1) = static;
    else
        str = fgetl(file);
        
        str = fgetl(file);
        moving_container(end+1).frame = [];
        moving_container(end).position = [];
        moving_container(end).projection = [];
%         moving.frame = [];
%         moving.position = [];
%         moving.projection = [];
        while ~isempty(str) && ischar(str)
            num = sscanf( str, '%d, (%f, %f, %f), (%f, %f)' );
            moving_container(end).frame(end+1) = num(1);
            moving_container(end).position(:,end+1) = num(2:4);
            moving_container(end).projection(:,end+1) = num(5:6);
            str = fgetl(file);
        end
		moving_container(end).pointIndex = ret(1) ;
%         moving_container(end+1) = moving;
    end
    
end

S.cam_container = cam_container;
S.static_container = static_container;
S.moving_container = moving_container;

fclose(file);