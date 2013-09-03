function [ output_args ] = outImage( handles )
%OUTIMAGE Summary of this function goes here
%  Detailed explanation goes here

%saveas( handles.figure_image, sprintf('%simg%03d.fig',  handles.outputPath, handles.frame_idx));
print(handles.figure_image,'-djpeg',sprintf('%simg%03d.jpg',  handles.outputPath, handles.frame_idx));

%F=getframe(handles.figure_image);
%imwrite(F.cdata, sprintf('%simg%03d.jpg',  handles.outputPath, handles.frame_idx));
