%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SLAM Summer School 2006, Oxford.
% Practical 3. SLAM using Monocular Vision.
% Practical exercise.
% J.M.M. Montiel, Javier Civera, Andrew J. Davison.
% {josemari, jcivera}@unizar.es, ajd@doc.ic.ac.uk
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [z, h, H_matched, R_matched, matched_all ] = matching( predicted, H_predicted, S, featuresInfo, std_pxl, static_container, moving_container, frame_idx )

% calculate the matching pixel for static feature
[matched_all]=matching_find( predicted, S, featuresInfo, static_container, moving_container, frame_idx );

matched_index = matched_all(:,1) ~= -1;

z = matched_all( matched_index, : );
z = reshape( z', size(z,1)*size(z,2), 1 );

z_size = size(z,1);

h = predicted(matched_index,:) ;
h = reshape(h',size(h,1)*size(h,2),1);

predicted_index = predicted(:,1) ~=-1;
H_matched_index = matched_index( predicted_index, : );
H_matched_index = reshape( [H_matched_index H_matched_index]', 2*size(H_matched_index,1), 1 );
H_matched = H_predicted( H_matched_index, : );

R_matched = eye( z_size )*std_pxl^2;

return ;



function [matched_all]=matching_find( predicted, S, featuresInfo, static_container, moving_container, frame_idx )

predicted_size = size( predicted, 1 );
matched_all = ones( predicted_size , 2 )*(-1);
for i=1:predicted_size
	if predicted(i,1) > 0
		point_idx = featuresInfo(i).positionInGroundTruth;
		C = S(1:2,1:2) ;
		S = S(3:end,3:end) ;
		
		K = chol(C)';
		if ( featuresInfo(i).type == 's' )
			index = find( static_container(point_idx).frame == frame_idx );
			if ~isempty(index) & isequal(static_container(point_idx).projection(:,index),[-1 -1])==0 & sum( [inv(K)*(static_container(point_idx).projection(:,index)-predicted(i,:)')].^2 ) < chi2inv(0.95,2)
				matched_all(i,:) = static_container(point_idx).projection(:,index)';
			end
		elseif ( featuresInfo(i).type == 'm' )
			index = find( moving_container(point_idx).frame == frame_idx );
			if ~isempty(index) & isequal(moving_container(point_idx).projection(:,index),[-1 -1])==0 & sum( [inv(K)*(moving_container(point_idx).projection(:,index)-predicted(i,:)')].^2 ) < chi2inv(0.95,2)
				matched_all(i,:) = moving_container(point_idx).projection(:,index)';
			end
		end
	end
end

return ;
