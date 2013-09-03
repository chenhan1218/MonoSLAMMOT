function [scrc,crc] = crosscorr(I1,I2,SVD)

% CROSSCORR Computes the normalized cross-correlation between two images
%
% [scrossc crossc] = crosscorr(I1,I2)
%
% input
%   I1,I2: Images
%
% output
%   scrossc: sum of the cross-correlations
%   crossc:  vector of cross-correlations
%
% NOTE: the correlation of 0_nxn and 0_nxn will be zero
%
% Prog: Diego
% Date: Jan 2002
%_____________________________________________________________________

%Check input parameters
sz = ones(1,3);
sz(1:ndims(I1)) = size(I1);
if prod(double((size(I2)==size(I1))))==0, %jmmm because prod didn't multipy logical values
  error(' ');
end;

%Compute the normalized cross-correlation
flag= 1; %See std for details
num = (I1-repmat(mean(mean(I1,1),2),sz(1:2))).*(I2-repmat(mean(mean(I2,1),2),sz(1:2)));
den = repmat(std(reshape(I1,[1,prod(sz(1:2)),sz(3)]),flag,2),sz(1:2))...
    .*repmat(std(reshape(I2,[1,prod(sz(1:2)),sz(3)]),flag,2),sz(1:2));
crc = (den~=0).*num./(den+(den==0));
scrc= reshape(mean(mean(crc,1),2),[1,size(crc,3)]);

return;