function [ ] = plotImage( axes_image, h_all, S, z_all, color1, color2, colorZ, chi2inv, featuresInfo )
%PLOTIMAGE Summary of this function goes here
%  Detailed explanation goes here

axes(axes_image);
title('Pred&Matched:Red. Pred&NOTMatched:Cian. Match:Green');

numFeatures = size(h_all,1);
for i=1:numFeatures
	if isequal(z_all(i,:),[-1 -1])==0
		plot( [ h_all(i,1) z_all(i,1) ], [ h_all(i,2) z_all(i,2) ], 'w-' );
		plot( z_all(i,1), z_all(i,2), 's', 'MarkerSize', 7,'MarkerFaceColor','b');
	end
	
	if h_all(i,:) ~= [-1 -1]
		if isequal(z_all(i,:),[-1 -1])==1
			color='c';
		elseif featuresInfo(i).estType=='s'
			color='b';
		elseif featuresInfo(i).estType=='m'
			color='r';
		else
			color='g';
		end
		
		plotUncertainEllip2D( S(1:2,1:2), h_all(i,:), chi2inv, color);
		plot( h_all(i,1), h_all(i,2), 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', color, 'MarkerSize', 3);
		text( h_all(i,1)+4, h_all(i,2), sprintf( '%d', featuresInfo(i).pointIndex ), 'color', color, 'FontSize', 13 );
		
		S = S(3:end,3:end);
	end
end
