function [ ] = plotImage( axes_image, h_all, S, z_all, color1, color2, colorZ, chi2inv, featuresInfo )
%PLOTIMAGE Summary of this function goes here
%  Detailed explanation goes here

axes(axes_image);
title('Pred&Matched:Red. Pred&NOTMatched:Cian. Match:Green');

numFeatures = size(h_all,1);
for i=1:numFeatures
	if featuresInfo(i).type=='m'
	end
	if isequal(z_all(i,:),[-1 -1])==0
		plot( [ h_all(i,1) z_all(i,1) ], [ h_all(i,2) z_all(i,2) ], 'w-' );
		if featuresInfo(i).estType == 's'
			plot( z_all(i,1), z_all(i,2), '+c', 'MarkerSize', 2);
			plotUncertainEllip2D( S(1:2,1:2), h_all(i,:), chi2inv, 'b');
		elseif featuresInfo(i).estType == 'm'
			plot( z_all(i,1), z_all(i,2), '+m', 'MarkerSize', 2);
			plotUncertainEllip2D( S(1:2,1:2), h_all(i,:), chi2inv, 'r');
		else
			plot( z_all(i,1), z_all(i,2), '+m', 'MarkerSize', 2);
			plotUncertainEllip2D( S(1:2,1:2), h_all(i,:), chi2inv, 'g');
		end
	end
	
	if h_all(i,:) ~= [-1 -1]
		if isequal(z_all(i,:),[-1 -1])==1
			plotUncertainEllip2D( S(1:2,1:2), h_all(i,:), chi2inv, color2);
			text( h_all(i,1)+4, h_all(i,2), sprintf( '%d', featuresInfo(i).pointIndex ), 'color', 'c', 'FontSize', 13 );
		elseif featuresInfo(i).estType=='s'
			plot( h_all(i,1), h_all(i,2), 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'b', 'MarkerSize', 3);
			text( h_all(i,1)+4, h_all(i,2), sprintf( '%d', featuresInfo(i).pointIndex ), 'color', 'b', 'FontSize', 13 );
		elseif featuresInfo(i).estType=='m'
			plot( h_all(i,1), h_all(i,2), 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'r', 'MarkerSize', 3);
			text( h_all(i,1)+4, h_all(i,2), sprintf( '%d', featuresInfo(i).pointIndex ), 'color', 'r', 'FontSize', 13 );
		else
			plot( h_all(i,1), h_all(i,2), 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g', 'MarkerSize', 3);
			text( h_all(i,1)+4, h_all(i,2), sprintf( '%d', featuresInfo(i).pointIndex ), 'color', 'g', 'FontSize', 13 );
		end
		S = S(3:end,3:end);
	end
end
