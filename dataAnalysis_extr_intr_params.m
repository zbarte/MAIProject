close all
clear

%%%%%%%%%%%%% EXP 1: EXTRACTING PARAMETERS FROM CALIBRATION %%%%%%%%%%%%%
load('cameraParams.mat');
load('estimationErrors.mat');

F = cameraParams.FocalLength;
f_x = F(1);
f_y = F(2);
principal_point = cameraParams.PrincipalPoint;
p_x = principal_point(1);
p_y = principal_point(2);
skew = cameraParams.Skew;

K = cameraParams.K;

% INITIAL GUESS OF WHERE CAMERA CENTRE IS RELATIVE TO WORLD (CF)
C_tilde = [0; 2300; 900];
x_cam = C_tilde(1);
y_cam = C_tilde(2);
z_cam = C_tilde(3);
phi_cam = 180;   % Rotation angle around Z axis
theta_cam = 0; % Rotation angle around Y axis
psi_cam = -90;   % Rotation angle around X axis

%%%%%%%%%%%%%%%%%%%%%%%%% EXPERIMENT 2: REAL DATA %%%%%%%%%%%%%%%%%%%%%%%%%
raw_data = readtable('EXPERIMENT_2_S1/aligned_data_ml_keyer.csv');

% Convert the first two columns from cell arrays to numeric arrays
% Check if the cells actually contain strings that need converting
if iscell(raw_data{:,2})
    numeric_col2 = str2double(raw_data{:,2});
else
    numeric_col2 = raw_data{:,2};
end

if iscell(raw_data{:,3})
    numeric_col3 = str2double(raw_data{:,3});
else
    numeric_col3 = raw_data{:,3};
end

% Create a logical index for rows where both of the first two columns are not zero
valid_rows = ~(numeric_col2 == 0 & numeric_col3 == 0);

% Use this logical index to filter the table
aligned_data = raw_data(valid_rows, :);

% extract data from file
time_exp2 = aligned_data.time;
x_cf = aligned_data.X;
y_cf = aligned_data.Y;
z_cf = aligned_data.Z;
cf_data = [x_cf * 1000, y_cf * 1000, z_cf * 1000]';
start_time = time_exp2(1);
relative_time = time_exp2 - start_time;
index = 1:numel(aligned_data.X);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OPTIMISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TRYING TO OPTIMISE BOTH INTRINSIC AND EXTRINSIC CAMERA PARAMETERS.
% E.G. REFINING MY CAMERA CALIBRATION AND ALSO FINDING THE OPTIMAL PLACE
% TO PUT THINGS

% data
pixel_x = aligned_data.x;
pixel_y = aligned_data.y;
pel_data = [pixel_x, pixel_y];

all_params = [f_x, f_y, p_x, p_y, x_cam, y_cam, z_cam, ... 
    phi_cam, theta_cam, psi_cam];
all_data = [cf_data', pel_data];

% CRAZYFLIE COORDINATES PROJECTED ONTO IMAGE PLANE
projected_points_exp2 = ProjectToCameraPlane(all_params, all_data);

% PLOTS CF POINTS TO IMAGE PLANE USING INITIAL PARAMETERS
% figure(1);
% plot(projected_points_exp2(1,:), projected_points_exp2(2,:), 'r.', 'LineWidth', 1);
% title('Projected Sensor Coordinates vs Ground Truth Coordinates');
% xlabel('X-coordinate (pixels)');
% ylabel('Y-coordinate (pixels)');
% grid on;
% set(gca, 'YDir', 'reverse');
% xlim([0, 3840]);
% ylim([0, 2160]);
% hold on;
% plot(pixel_x, pixel_y, 'b.', 'LineWidth', 1);
% legend('Crazyflie Image Projections', 'Ground Truth Image Coordinates', Location='southwest');
% drawnow;

% ERROR CALCULATIONS
% MAIN ERROR
mean_euclidean_distance_error = MeanEuclideanDistanceError(all_params, all_data);
fprintf('Mean Euclidean distance error: %.2f\n', mean_euclidean_distance_error);

% OTHER ERRORS
mean_absolute_error = MeanAbsoluteError(all_params, all_data);
fprintf('Mean absolute error: %.2f\n', mean_absolute_error);
mean_squared_error = MeanSquaredError(all_params, all_data);
fprintf('Mean squared error: %.2f\n', mean_squared_error);

% alpha trimmed mean
alpha = 0.1;
fprintf('Alpha value: %.1f\n', alpha);

% ALPHA TRIMMED MEAN FOR MED
alpha_med = AlphaTrimmedMeanEuclideanDistanceError(all_params, all_data, alpha);
fprintf('Alpha-trimmed mean Euclidean distance error: %.2f\n', alpha_med);

% OTHER ALPHA TRIMMED
alpha_mae = AlphaTrimmedMeanAbsoluteError(all_params, all_data, alpha);
fprintf('Alpha-trimmed mean absolute error: %.2f\n', alpha_mae);
alpha_mse = AlphaTrimmedMeanSquaredError(all_params, all_data, alpha);
fprintf('Alpha-trimmed mean squared error: %.2f\n', alpha_mse);

% OPTIMISATION
optimal_MED_params = fminsearch(@(params) AlphaTrimmedMeanEuclideanDistanceError(params, all_data, alpha), all_params);
optimal_MAE_params = fminsearch(@(params) AlphaTrimmedMeanAbsoluteError(params, all_data, alpha), all_params);
optimal_MSE_params = fminsearch(@(params) AlphaTrimmedMeanSquaredError(params, all_data, alpha), all_params);

fprintf('Initial parameters: \n');
fprintf('%.2f ', all_params);
fprintf('\n');
% MED
fprintf('\nOptimised MED parameters: \n');
fprintf('%.2f ', optimal_MED_params);
fprintf('\n');
% OTHERS
fprintf('\nOptimised MAE parameters: \n');
fprintf('%.2f ', optimal_MAE_params);
fprintf('\n');
fprintf('\nOptimised MSE parameters: \n');
fprintf('%.2f ', optimal_MSE_params);
fprintf('\n');


% COMPUTE MINIMISED ERRORS
min_mean_euclidean_distance_error = AlphaTrimmedMeanEuclideanDistanceError(optimal_MED_params, all_data, alpha);
fprintf('Optimal mean Euclidean distance error: %.2f\n', min_mean_euclidean_distance_error);
% OTHER ONES
min_mean_absolute_error = AlphaTrimmedMeanAbsoluteError(optimal_MAE_params, all_data, alpha);
fprintf('Optimal mean absolute error: %.2f\n', min_mean_absolute_error);
min_mean_squared_error = AlphaTrimmedMeanSquaredError(optimal_MSE_params, all_data, alpha);
fprintf('Optimal mean squared error: %.2f\n', min_mean_squared_error);

med_projected_points = ProjectToCameraPlane(optimal_MED_params, all_data);

% PLOTS CF POINTS TO IMAGE PLANE USING MED OPTIMISED PARAMETERS
figure(2);
plot(med_projected_points(1,:), med_projected_points(2,:), 'r.', 'LineWidth', 1);
title('Projected Sensor Coordinates (min. MED) vs Ground Truth Coordinates');
xlabel('X-coordinate (pixels)');
ylabel('Y-coordinate (pixels)');
grid on;
set(gca, 'YDir', 'reverse');
xlim([0, 3840]);
ylim([0, 2160]);
hold on;
plot(pel_data(:,1), pel_data(:,2), 'b.', 'LineWidth', 1);
legend('Crazyflie Image Projections', 'Ground Truth Image Coordinates', Location='southwest');
drawnow;

%%%%%%%%%%%%%%%%%%%%%%% EXPERIMENT 3: GROUND TRUTH %%%%%%%%%%%%%%%%%%%%%%%

% folder of crazyflie readings
folder = 'EXPERIMENT_3_S1';
 
% list of csv files
fileList = dir(fullfile(folder, '*.csv'));
 
% initialized cell array
mean_sensor_readings = cell(length(fileList), 1);
 
 
for i = 1:length(fileList)
    filename = fullfile(folder, fileList(i).name);
 
    big_bro = readtable(filename);

    x = big_bro.x;
    y = big_bro.y;
    z = big_bro.z;

    mean_x = mean(x) * 1000;
    mean_y = mean(y) * 1000;
    mean_z = mean(z) * 1000;
    mean_position = [mean_x, mean_y, mean_z];

    % storing the mean position in cell array
    mean_sensor_readings{i} = mean_position;
end
 
final_sensor_readings = cell2mat(mean_sensor_readings);
exp3_projected_points = ProjectToCameraPlane(optimal_MED_params, final_sensor_readings);

% Experiment 3 ground truth pixel coordinates obtained manually
ground_truth_x = [1997 2083 3410 236 750 1996 2928 678 461 3623];
ground_truth_y = [813 788 791 820 275 488 468 497 1602 1542];
ones_row = ones(1, numel(ground_truth_x));
ground_truth_image_coords = [ground_truth_x; ground_truth_y; ones_row];

% EXP 3: PROJECTING GROUND TRUTH PIXELS BACK TO 3D
projection_back_to_3d = ProjectImageTo3D(all_params, ground_truth_image_coords);
MEDopt_projection_back_to_3d = ProjectImageTo3D(optimal_MED_params, ground_truth_image_coords);

measured_gt_x = [0 0 -1200 1500 840 0 -1040 1440 980 -1010];
measured_gt_y = [-400 600 0 0 420 -700 -570 -640 600 600];
measured_gt_z = [900 900 900 900 1270 1270 1270 1270 430 430];
measured_ground_truth = [measured_gt_x; measured_gt_y; measured_gt_z];

% just checking reprojection errors
exp3_min_reproj_errors = mean(sqrt(sum((MEDopt_projection_back_to_3d - measured_ground_truth).^2)));
exp3_reproj_errors = mean(sqrt(sum((projection_back_to_3d - measured_ground_truth).^2)));

% EXP 3: PLOTTING PROJECTED GT PIXELS (OBTAINED WITH AND WITHOUT USING
% OPTIMAL PARAMS) WITH AVERAGE CF READINGS
figure;
plot3(MEDopt_projection_back_to_3d(1, :), MEDopt_projection_back_to_3d(2, :), MEDopt_projection_back_to_3d(3, :), ...
    'o', 'MarkerSize', 5, 'MarkerFaceColor', 'r'); % MAE OPTIMISED 2D TO 3D
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;
plot3(projection_back_to_3d(1, :), projection_back_to_3d(2, :), projection_back_to_3d(3, :), ...
    'o', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % 3D NON-OPTIMISED 2D TO 3D
% plot3(final_sensor_readings(:, 1), final_sensor_readings(:, 2), final_sensor_readings(:, 3), ... 
%     'o', 'MarkerSize', 5, 'MarkerFaceColor', 'b'); % CF READINGS
plot3(measured_ground_truth(1, :), measured_ground_truth(2, :), measured_ground_truth(3, :), ...
    'o', 'MarkerSize', 5, 'MarkerFaceColor', 'k');
title('Plot of GT Reprojections vs GT Measurements');
legend('MED Opt. Params.', 'Initial Params.', 'Ground Truth');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function K = createCameraMatrix(f_x, f_y, p_x, p_y)
    K = [f_x 0 p_x; 
         0 f_y p_y; 
         0 0 1];
end

function R = rotationMatrixZYX(phi, theta, psi)
    % degrees to radians
    phi = deg2rad(phi);
    theta = deg2rad(theta);
    psi = deg2rad(psi);
    
    % computes sine and cosine of angles
    c1 = cos(phi);
    s1 = sin(phi);
    c2 = cos(theta);
    s2 = sin(theta);
    c3 = cos(psi);
    s3 = sin(psi);
    
    % rotation matrix
    R = [c1*c2, c1*s2*s3 - c3*s1, s1*s3 + c1*c3*s2;
         c2*s1, c1*c3 + s1*s2*s3, c3*s1*s2 - c1*s3;
         -s2, c2*s3, c2*c3];
end

function projected_points = ProjectToCameraPlane(params, data)
    f_x = params(1);
    f_y = params(2);
    p_x = params(3);
    p_y = params(4);
    x_cam = params(5);
    y_cam = params(6);
    z_cam = params(7);
    phi_cam = params(8);
    theta_cam = params(9);
    psi_cam = params(10);

    cf_data = data(:, 1:3)';
    % pel_data = data(:, 4:5)';
    num_points = size(cf_data, 2);
    
    R = rotationMatrixZYX(phi_cam, theta_cam, psi_cam);
    K = createCameraMatrix(f_x, f_y, p_x, p_y);
    C_tilde = [x_cam; y_cam; z_cam];

    t = -R * C_tilde;

    % store projected points
    projected_points = zeros(3, num_points);

    for i = 1:num_points
        % Project 3D point onto image plane
        projected_p_homogeneous = K * [R, t] * [cf_data(:, i); 1];

        projected_points(:, i) = projected_p_homogeneous ./ projected_p_homogeneous(3); % homogeneous to cartesian 
    end
end

% all_params = [f_x, f_y, p_x, p_y, x_cam, y_cam, z_cam, ... 
%     phi_cam, theta_cam, psi_cam];
function projection_back_to_3d = ProjectImageTo3D(params, projected_points)
    f_x = params(1); % CHANGE TO INCLUDE UNKNOWN PARAMS
    f_y = params(2);
    p_x = params(3);
    p_y = params(4);
    x_cam = params(5);
    y_cam = params(6);
    z_cam = params(7);
    phi_cam = params(8);
    theta_cam = params(9);
    psi_cam = params(10);

    % cf_data = data(:, 1:3)';

    % Initialize matrix to store all 3D coordinates
    projection_back_to_3d = zeros(3, size(projected_points, 2));
    
    R = rotationMatrixZYX(phi_cam, theta_cam, psi_cam);
    K = createCameraMatrix(f_x, f_y, p_x, p_y);
    C_tilde = [x_cam; y_cam; z_cam];
    t = -R * C_tilde;

    % Construct projection matrix P
    P = K * [R, t];
    
    % Compute the pseudo-inverse of P
    P_inv = pinv(P);
    
    % Iterate over each column of the input matrix projected_points
    for i = 1:size(projected_points, 2)
        % Extract current 2D coordinates
        x = projected_points(:, i);
        
        % Apply the inverse transformation to x
        X = P_inv * x;
        
        % Convert from homogeneous to Cartesian coordinates
        X_cartesian = X(1:3) ./ X(4);
        
        % Store the result in the output matrix
        projection_back_to_3d(:, i) = X_cartesian;
    end
end

function MED = MeanEuclideanDistanceError(params, data)
    f_x = params(1);
    f_y = params(2);
    p_x = params(3);
    p_y = params(4);
    x_cam = params(5);
    y_cam = params(6);
    z_cam = params(7);
    phi_cam = params(8);
    theta_cam = params(9);
    psi_cam = params(10);

    cf_data = data(:, 1:3)';
    pel_data = data(:, 4:5)';

    num_points = size(cf_data, 2); % number of points
    errors_matrix = zeros(1, num_points); % stores errors

    R = rotationMatrixZYX(phi_cam, theta_cam, psi_cam);
    K = createCameraMatrix(f_x, f_y, p_x, p_y);
    C_tilde = [x_cam; y_cam; z_cam];

    t = -R * C_tilde;

    % label = (pel_data(1,:) ~= 0);
    % index = find(label == 1);

    % errors_matrix = zeros(1, length(index)); % stores errors

    for i = 1:num_points
        % i = index(n);
        proj_cf_data_homogen = K * [R, t] * [cf_data(:, i); 1];
        proj_cf_data = proj_cf_data_homogen(1:2) ./ proj_cf_data_homogen(3);

        % errors_matrix(n) = sum(abs(proj_cf_data(1:2) - pel_data(:, i)));
        errors_matrix(i) = sqrt((proj_cf_data(1) - pel_data(1,i))^2 + (proj_cf_data(2) - pel_data(2,i))^2);
    end

    MED = mean(errors_matrix);
end

function alpha_MED = AlphaTrimmedMeanEuclideanDistanceError(params, data, alpha)
    f_x = params(1);
    f_y = params(2);
    p_x = params(3);
    p_y = params(4);
    x_cam = params(5);
    y_cam = params(6);
    z_cam = params(7);
    phi_cam = params(8);
    theta_cam = params(9);
    psi_cam = params(10);

    cf_data = data(:, 1:3)';
    pel_data = data(:, 4:5)';

    num_points = size(cf_data, 2); % number of points
   

    R = rotationMatrixZYX(phi_cam, theta_cam, psi_cam);
    K = createCameraMatrix(f_x, f_y, p_x, p_y);
    C_tilde = [x_cam; y_cam; z_cam];

    t = -R * C_tilde;

    % transforms cf data from cf system to world coord system using params
    % 11 onwards, should be no difference though bc i defined cf and 
    % world origins as the same
    % world_cf_data = transformPointsToWorld(cf_data, params);
    % label = (pel_data(1,:) ~= 0);
    % index = find(label == 1);

    errors_matrix = zeros(1, num_points); % stores errors
    
    for i = 1:num_points
        % i = index(n);
        proj_cf_data_homogen = K * [R, t] * [cf_data(:, i); 1];
        proj_cf_data = proj_cf_data_homogen(1:2) ./ proj_cf_data_homogen(3);

        % errors_matrix(i) = sum(abs(proj_cf_data(1:2) - pel_data(:, i)));
        errors_matrix(i) = sqrt((proj_cf_data(1) - pel_data(1,i))^2 + (proj_cf_data(2) - pel_data(2,i))^2);
    end

    % Apply alpha-trimmed mean
    sorted_errors = sort(errors_matrix);
    trimmed_errors = sorted_errors(floor(alpha*num_points)+1 : end-floor(alpha*num_points));
    alpha_MED = mean(trimmed_errors);
    % mae = mean(errors_matrix);
end

% all_params = [f_x, f_y, p_x, p_y, x_cam, y_cam, z_cam, ... 
%     phi_cam, theta_cam, psi_cam];

%%%%%%%%% MEAN ABSOLUTE ERROR!!!
function mae = MeanAbsoluteError(params, data)
    f_x = params(1);
    f_y = params(2); %xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    p_x = params(3);
    p_y = params(4);
    x_cam = params(5);
    y_cam = params(6);
    z_cam = params(7);
    phi_cam = params(8);
    theta_cam = params(9);
    psi_cam = params(10);

    cf_data = data(:, 1:3)';
    pel_data = data(:, 4:5)';

    num_points = size(cf_data, 2); % number of points

    R = rotationMatrixZYX(phi_cam, theta_cam, psi_cam);
    K = createCameraMatrix(f_x, f_y, p_x, p_y);
    C_tilde = [x_cam; y_cam; z_cam];

    t = -R * C_tilde;

    % transforms cf data from cf system to world coord system using params
    % 11 onwards, should be no difference though bc i defined cf and 
    % world origins as the same
    % world_cf_data = transformPointsToWorld(cf_data, params);
  
    errors_matrix = zeros(1, num_points); % stores errors

    % for i = 1:num_points
    %     proj_cf_data_homogen = K * [R, t] * [world_cf_data(:, i); 1];
    %     proj_cf_data = proj_cf_data_homogen ./ proj_cf_data_homogen(3);
    % 
    %     errors_matrix(i) = sum(abs(proj_cf_data(1:2) - pel_data(:, i)));
    % end
    for i = 1:num_points
        proj_cf_data_homogen = K * [R, t] * [cf_data(:, i); 1];
        proj_cf_data = proj_cf_data_homogen ./ proj_cf_data_homogen(3);

        errors_matrix(i) = sum(abs(proj_cf_data(1:2) - pel_data(:, i)));
    end
    mae = mean(errors_matrix);
end

function alpha_mae = AlphaTrimmedMeanAbsoluteError(params, data, alpha)
    f_x = params(1);
    f_y = params(2); %xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    p_x = params(3);
    p_y = params(4);
    x_cam = params(5);
    y_cam = params(6);
    z_cam = params(7);
    phi_cam = params(8);
    theta_cam = params(9);
    psi_cam = params(10);

    cf_data = data(:, 1:3)';
    pel_data = data(:, 4:5)';

    num_points = size(cf_data, 2); % number of points
   

    R = rotationMatrixZYX(phi_cam, theta_cam, psi_cam);
    K = createCameraMatrix(f_x, f_y, p_x, p_y);
    C_tilde = [x_cam; y_cam; z_cam];

    t = -R * C_tilde;

    % transforms cf data from cf system to world coord system using params
    % 11 onwards, should be no difference though bc i defined cf and 
    % world origins as the same
    % world_cf_data = transformPointsToWorld(cf_data, params);

    errors_matrix = zeros(1, num_points); % stores errors
    
    for i = 1:num_points
        proj_cf_data_homogen = K * [R, t] * [cf_data(:, i); 1];
        proj_cf_data = proj_cf_data_homogen ./ proj_cf_data_homogen(3);

        errors_matrix(i) = sum(abs(proj_cf_data(1:2) - pel_data(:, i)));
        % errors_matrix(n) = sqrt((proj_cf_data(1) - pel_data(1,i))^2 + (proj_cf_data(2) - pel_data(2,i))^2);
    end

    % Apply alpha-trimmed mean
    sorted_errors = sort(errors_matrix);
    trimmed_errors = sorted_errors(floor(alpha*num_points)+1 : end-floor(alpha*num_points));
    alpha_mae = mean(trimmed_errors);

end

%%%%%%%%%%%%%%%%%% MEAN SQUARED ERROR
function mse = MeanSquaredError(params, data)
    f_x = params(1);
    f_y = params(2); %xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    p_x = params(3);
    p_y = params(4);
    x_cam = params(5);
    y_cam = params(6);
    z_cam = params(7);
    phi_cam = params(8);
    theta_cam = params(9);
    psi_cam = params(10);

    cf_data = data(:, 1:3)';
    pel_data = data(:, 4:5)';

    num_points = size(cf_data, 2); % number of points

    R = rotationMatrixZYX(phi_cam, theta_cam, psi_cam);
    K = createCameraMatrix(f_x, f_y, p_x, p_y);
    C_tilde = [x_cam; y_cam; z_cam];

    t = -R * C_tilde;

    % transforms cf data from cf system to world coord system using params
    % 11 onwards, should be no difference though bc i defined cf and 
    % world origins as the same
    % world_cf_data = transformPointsToWorld(cf_data, params);
  
    errors_matrix = zeros(1, num_points); % stores errors

    for i = 1:num_points
        
        proj_cf_data_homogen = K * [R, t] * [cf_data(:, i); 1];
        proj_cf_data = proj_cf_data_homogen ./ proj_cf_data_homogen(3);

        
        errors_matrix(i) = sum((proj_cf_data - [pel_data(:, i); 1]).^2);
    end
    mse = mean(errors_matrix);
end

function alpha_mse = AlphaTrimmedMeanSquaredError(params, data, alpha)
    f_x = params(1);
    f_y = params(2); %xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
    p_x = params(3);
    p_y = params(4);
    x_cam = params(5);
    y_cam = params(6);
    z_cam = params(7);
    phi_cam = params(8);
    theta_cam = params(9);
    psi_cam = params(10);

    cf_data = data(:, 1:3)';
    pel_data = data(:, 4:5)';

    num_points = size(cf_data, 2); % number of points


    R = rotationMatrixZYX(phi_cam, theta_cam, psi_cam);
    K = createCameraMatrix(f_x, f_y, p_x, p_y);
    C_tilde = [x_cam; y_cam; z_cam];

    t = -R * C_tilde;

    % transforms cf data from cf system to world coord system using params
    % 11 onwards, should be no difference though bc i defined cf and 
    % world origins as the same
    % world_cf_data = transformPointsToWorld(cf_data, params);

    errors_matrix = zeros(1,num_points); % stores errors

    for i = 1:num_points
        proj_cf_data_homogen = K * [R, t] * [cf_data(:, i); 1];
        proj_cf_data = proj_cf_data_homogen ./ proj_cf_data_homogen(3);

        % errors_matrix(i) = sum(abs(proj_cf_data(1:2) - pel_data(:, i)));
        errors_matrix(i) = sum((proj_cf_data - [pel_data(:, i); 1]).^2);
    end

    % Apply alpha-trimmed mean
    sorted_errors = sort(errors_matrix);
    trimmed_errors = sorted_errors(floor(alpha*num_points)+1 : end-floor(alpha*num_points));
    alpha_mse = mean(trimmed_errors);
end