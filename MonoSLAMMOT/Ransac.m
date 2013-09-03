function [ max_inliers, H_matching, R_matching, z, h ] = Ransac( step, z, h, H_matching, R_matching, z_all, h_all, H_predicted, x_km1_k, p_km1_k, cam, std_pxl )
%RANSAC Summary of this function goes here
%  Detailed explanation goes here

z_size = size(z,1)/2;

num = 2;
sel_R = eye( num )*std_pxl^2;
% sel_R = eye( num*2 )*R_matching(1,1);
z_index = z_all(:,1) ~= -1;
threshold = 0.69;
max_inliers = [];
numFeatures = (size(x_km1_k,1) - 13)/6;
average_dis = 100;

% if step ~= 2 
    max_inliers = z_index;
    z = z_all( max_inliers, : );
    z = reshape( z', size(z,1)*size(z,2), 1 );
    z = z-0.5*ones( size(z,1), 1 );
    
    h = h_all( max_inliers, : );
    h = reshape(h', size(h,1)*size(h,2), 1 );
    
    predicted_index = h_all(:,1) ~= -1;
    predicted_inlier = max_inliers( predicted_index );
    predicted_inlier = reshape( [predicted_inlier predicted_inlier]', 2*size(predicted_inlier,1), 1);
    H_matching = H_predicted( predicted_inlier, : );
    R_matching = eye( size(z,1) )*std_pxl^2;
    return;
% else 
%     max_inliers = z_index;
% %     max_inliers(9:11,1) = logical(zeros(3,1)); % synth7
%     max_inliers(1:3,1) = logical(zeros(3,1)); % synth10
% %     max_inliers(14:16,1) = logical(zeros(3,1)); % synth11
%     z = z_all( max_inliers, : );
%     z = reshape( z', size(z,1)*size(z,2), 1 );
%     z = z-0.5*ones( size(z,1), 1 );
%     
%     h = h_all( max_inliers, : );
%     h = reshape(h', size(h,1)*size(h,2), 1 );
%     
%     predicted_index = h_all(:,1) ~= -1;
%     predicted_inlier = max_inliers( predicted_index );
%     predicted_inlier = reshape( [predicted_inlier predicted_inlier]', 2*size(predicted_inlier,1), 1);
%     H_matching = H_predicted( predicted_inlier, : );
%     R_matching = eye( size(z,1) )*std_pxl^2;
%     return;
% end

for round=1:1500
    sel = unique(ceil(rand(num,1)*z_size));
    while ( size(sel,1) < num )
          sel = unique(vertcat(sel, ceil(rand(num-size(sel),1)*z_size)));
    end
    
    index = vertcat( sel*2, sel*2-1 );
    sel_z = z(index);
    sel_h = h(index);
    sel_H = H_matching(index,:);
    
    [ x_k_k, p_k_k ] = update( x_km1_k, p_km1_k, sel_H, sel_R, sel_z, sel_h );
    
    xv_k_k = x_k_k( 1:13 );
    XfeatToProject = x_km1_k( 14:end );
    h_new = ones( numFeatures, 2 )*(-1);
    for i = 1:numFeatures
        yi = XfeatToProject( 1:6 );
        XfeatToProject = XfeatToProject( 7:end );
        zi = hi ( yi, xv_k_k, cam );
        if (~isempty(zi))
            h_new(i,:)=zi';
        end
    end
    
    h_index = h_new(:,1) ~= -1;
    and_index = h_index & z_index;
%     valid_h = h_new(and_index,:);
%     valid_z = z_all(and_index,:);
%     diff = valid_h - valid_z;
    diff = h_new - z_all;
    
    z_invalid = z_all(:,1) == -1;
    diff(z_invalid,:) = [ 0 ];
    
    dis = sqrt(diff(:,1).^2 + diff(:,2).^2);
    inlier = dis(:,1) < threshold;
    inlier = inlier & and_index;
%     inliers = 1:size(inlier_index);
%     inliers = inliers(inlier_index);
%     if ( size(inliers) > size(max_inliers) )
    if ( size(find(inlier),1) > size(find(max_inliers),1) && sum(dis(inlier,1))/size(find(inlier),1) < average_dis)
        max_inliers = inlier;
        average_dis = sum(dis)/size(dis,1);
    end
%     if ( size(max_inliers,2) == numFeatures )
    if ( size(find(max_inliers),1) >= z_size )
        break;
    end
end

z = z_all( max_inliers, : );
z = reshape( z', size(z,1)*size(z,2), 1 );
z = z-0.5*ones( size(z,1), 1 );

h = h_all( max_inliers, : );
h = reshape(h', size(h,1)*size(h,2), 1 );

predicted_index = h_all(:,1) ~= -1;
predicted_inlier = max_inliers( predicted_index );
predicted_inlier = reshape( [predicted_inlier predicted_inlier]', 2*size(predicted_inlier,1), 1);
H_matching = H_predicted( predicted_inlier, : );
R_matching = eye( size(z,1) )*R_matching(1,1);

% % matched_index = matched_all(:,1) ~= -1;
% %     
% % z = matched_all( matched_index, : ) ;
% % z = reshape( z',size(z,1)*size(z,2), 1 );
% % z = z-0.5*ones( size(z,1), 1 );
% % 
% % z_size = size( z, 1 );
% % 
% % h = predicted(matched_index,:) ;
% % h = reshape(h',size(h,1)*size(h,2),1);
% % 
% % predicted_index = predicted(:,1) ~=-1;
% % H_matched_index = matched_index( predicted_index, : );
% % H_matched_index = reshape( [H_matched_index H_matched_index]', 2*size(H_matched_index,1), 1 );
% % H_matched = H_predicted( H_matched_index, : );
% % 
% % R_matched = eye( z_size )*std_pxl^2;


% % function [ x_k_k, p_k_k ] = update( x_km1_k, p_km1_k, H, R, z, h )
% 
% 
% % numFeatures = (size( x_km1_k, 1 ) - 13)/6;
% % XfeatToPredict = x_km1_k( 14:end );
% % h_all = ones( numFeatures, 2 )*(-1);
% % H = [];
% % 
% % % Extract camera motion parameters
% % xv_km1_k = x_km1_k( 1:13 );
% % 
% % for i = 1:numFeatures
% %     
% %     % Extract feature
% %     yi = XfeatToPredict( 1:6 );
% %     XfeatToPredict = XfeatToPredict( 7:end );
% %     % Measurement equation
% %     zi = hi( yi, xv_km1_k, cam );
% %     if (~isempty(zi)) % if is predicted in the image
% %         h_all(i,:) = zi';
% %         % Calculate derivatives
% %         Hi = calculate_Hi(  xv_km1_k, yi, zi, cam, i, numFeatures);
% %         H = [ H; Hi];
% %     end
% %    
% % end
