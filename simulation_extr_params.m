close all
clear

% Boundaries of image plane
Y_min = 0;
Y_max = 100;
Z_min = 0;
Z_max = 100;

% Mesh grid for y and z coordinates
[Y, Z] = meshgrid(Y_min:10:Y_max, Z_min:10:Z_max);

% Define constant x coordinates
focal_length = 50;
X = ones(size(Y)) * focal_length;

% camera focal point (origin) (world coordinates)
x_f = 0;
y_f = 50;
z_f = 50;
focal_point = [x_f, y_f, z_f];

% rotation matrix angles
phi = 0;   % Rotation angle around Z axis
theta = -90; % Rotation angle around Y axis
psi = -90;   % Rotation angle around X axis

% creates array of 1000 points with random noise and standard deviation
num_points = 1000;
noise_std = 1.1; % noise standard deviation 
points = generateRandomPointsWithNoise(num_points, noise_std);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Generate Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
hold on
title('Pinhole Camera Model with Synthetic Data');
surf(X, Y, Z);
plot3(x_f, y_f, z_f, 'ro') % camera focal point/origin (world)
scatter3(points(:,1), points(:,2), points(:,3), 'm.');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
xlim([0 200]);
ylim([0 100]);
zlim([0 100]);
view(3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CF R+T %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x_cf = 100.5;
y_cf = 46.8;
z_cf = 0.7;
cf_origin = [x_cf; y_cf; z_cf];
cf_origin_noisy = cf_origin + randn(size(cf_origin)) * 0.1;

% plot new cf origin
plot3(x_cf, y_cf, z_cf, 'ko');

phi_cf = 0+29;   % Rotation angle around Z axis
theta_cf = -90+180; % Rotation angle around Y axis
psi_cf = -90-13;   % Rotation angle around X axis

% WE ARE ASSUMING IN THIS SIMULATION THAT THE NUMBER OF PIXELS PER UNIT
% DISTANCE IN IMAGE COORDS IS 1 IN X AND Y DIRECTION I.E. NOT A CCD CAMERA,
% IN ACTUALITY WE WILL BE USING A CCD CAMERA SO IT'S NOT F IN THE K MATRIX

% parameters from new CF origin
% all_params = [focal_length, y_f, z_f, phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf];
all_params = [phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf];
cam_params = [focal_length, y_f, z_f];

% Transforms points into cf system (in reality we would not know the
% transformation but would only know these measurements from the cf)
X_vectors = points';
points_in_cf = generateRandomPointsInCf(X_vectors, all_params);
points_in_cf_trans = points_in_cf';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Optimisation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% same as testing all_params but testing how far off initial guess can be.
initial_params = all_params + 10*rand(size(all_params));

x_cartesian = projectPointsCartesianWithNoise(initial_params, cam_params, points_in_cf_trans, noise_std);  
data = [points_in_cf, x_cartesian'];

% compute mean squared error
mse = computeMSError(all_params, cam_params, data);
fprintf('Mean squared error: %.2f\n', mse);

% compute mean absolute error
mae = computeMAError(all_params, cam_params, data);
fprintf('Mean absolute error: %.2f\n', mae);

% compute mean absolute error
med = computeMEDError(all_params, cam_params, data);
fprintf('Mean Euclidean distance error: %.2f\n', med);

optimised_MSE_params = fminsearch(@(params) computeMSError(params, cam_params, data), all_params);
optimised_MAE_params = fminsearch(@(params) computeMAError(params, cam_params, data), all_params);
optimised_MED_params = fminsearch(@(params) computeMEDError(params, cam_params, data), all_params);

fprintf('Initial parameters: \n');
fprintf('%.2f ', initial_params);
fprintf('\n');
fprintf('\nOptimal MSE parameters: \n');
fprintf('%.2f ', optimised_MSE_params);
fprintf('\n');
fprintf('\nOptimal MAE parameters: \n');
fprintf('%.2f ', optimised_MAE_params);
fprintf('\n');
fprintf('\nOptimal MED parameters: \n');
fprintf('%.2f ', optimised_MED_params);
fprintf('\n');


% compute optimised mean squared error
mse_optimised = computeMSError(optimised_MSE_params, cam_params, data);
fprintf('Optimal mean squared error: %.2f\n', mse_optimised);

% compute optimised mean absolute error
mae_optimised = computeMAError(optimised_MAE_params, cam_params, data);
fprintf('Optimal mean absolute error: %.2f\n', mae_optimised);

% compute optimised mean absolute error
med_optimised = computeMEDError(optimised_MED_params, cam_params, data);
fprintf('Optimal mean Euclidean distance error: %.2f\n', med_optimised);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calculates rotation matrix
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

% calculates camera matrix
function K = createCameraMatrix(focal_length, y_f, z_f)
    K = [focal_length 0 y_f; 
         0 focal_length z_f; 
         0 0 1];
end

% Generates points (in cf coordinates) with noise
function points = generateRandomPointsWithNoise(num_points, noise_std)
    % Initialize array to store points
    points = zeros(num_points, 3);

    % Define constraints
    min_x = 51;
    max = 100;
    max_x = 200;

    % Generate random points
    for i = 1:num_points
        % Random coordinates
        x_r = min_x + (max_x - min_x) * rand;
        y_r = max * rand;
        z_r = max * rand;
        
        % Add noise using randn
        x_noise = noise_std * randn;
        y_noise = noise_std * randn;
        z_noise = noise_std * randn;
        
        % Ensure cf location's x-coordinate is past image plane
        x_r = x_r + x_noise;
        y_r = y_r + y_noise;
        z_r = z_r + z_noise;
        
        % array of points
        points(i, :) = [x_r, y_r, z_r];
    end
end

% all_params = [phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf];
% camera_params = [focal_length, y_f, z_f];
function points_in_cf = generateRandomPointsInCf(points, all_params)
    % Extract CF parameters
    % x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf
    x_cf = all_params(4);
    y_cf = all_params(5);
    z_cf = all_params(6);
    phi_cf = all_params(7);
    theta_cf = all_params(8);
    psi_cf = all_params(9);
    
    % random points in original coordinate system
    % points = generateRandomPointsWithNoise(num_points, noise_std);
    
    % rotation matrix in cf system
    R_cf = rotationMatrixZYX(phi_cf, theta_cf, psi_cf);

    % Apply rotation to points
    points_rotated = (R_cf * points)';

    % Translate points to CF coordinate system
    points_in_cf = points_rotated + [x_cf, y_cf, z_cf];
end

% all_params = [phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf];
% camera_params = [focal_length, y_f, z_f];
function og_X_vectors = transformPointsToOriginal(X_vectors_cf, params)
    % Extract CF parameters
    x_cf = params(4);
    y_cf = params(5);
    z_cf = params(6);
    phi_cf = params(7);
    theta_cf = params(8);
    psi_cf = params(9);
    
    % Calculate rotation matrix from CF to original coordinate system
    R_cf = rotationMatrixZYX(phi_cf, theta_cf, psi_cf);
    
    % Calculate inverse of the rotation matrix
    % R_cf_inv = inv(R_cf);
    
    % Initialize array to store transformed points
    num_points = size(X_vectors_cf, 2);
    og_X_vectors = zeros(3, num_points);
    
    % Loop through each point and transform it
    for i = 1:num_points
        % Shift the point by the CF origin
        X_vector_cf_shifted = X_vectors_cf(:, i) - [x_cf; y_cf; z_cf];
        
        % Transform the point back to the original coordinate system
        og_X_vectors(:, i) = R_cf \ X_vector_cf_shifted;
    end
end


% finds image coordinates of generated points (also with noise)
% all_params = [phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf];
% camera_params = [focal_length, y_f, z_f];
function x_cartesian = projectPointsCartesianWithNoise(params, cam_params, X_vectors, noise_std)    
    num_points = size(X_vectors, 2); 
    
    focal_length = cam_params(1);
    y_f = cam_params(2);
    z_f = cam_params(3);
    phi = params(1);
    theta = params(2);
    psi = params(3);
    % x_cf = params(7);
    % y_cf = params(8);
    % z_cf = params(9);
    % phi_cf = params(10);
    % theta_cf = params(11);
    % psi_cf = params(12);

    % stores cartesian coordinates of projected points
    x_cartesian = zeros(3, num_points);
    
    % calculates rotation matrix
    R = rotationMatrixZYX(phi, theta, psi);
    K = createCameraMatrix(focal_length, y_f, z_f);
    % make up I_tildeC
    C_vector = [0; y_f; z_f];
    I_tildeC = [eye(3), -C_vector];
    
    % transform X_vecs back to original
    og_X_vectors = transformPointsToOriginal(X_vectors, params);

    % Project each point onto the image plane
    for i = 1:num_points
        % Project 3D point onto image plane
        x_homogeneous = K * R * I_tildeC * [og_X_vectors(:, i); 1];
        
        x_cartesian(:, i) = x_homogeneous ./ x_homogeneous(3); % homogeneous to cartesian
        
        noise = noise_std * randn(3, 1); % adds noise
        x_cartesian(:, i) = x_cartesian(:, i) + noise;
    end
end

% all_params = [phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf];
% camera_params = [focal_length, y_f, z_f];
function mse = computeMSError(params, cam_params, data)
    focal_length = cam_params(1);
    y_f = cam_params(2);
    z_f = cam_params(3);
    phi = params(1);
    theta = params(2);
    psi = params(3);
    % x_cf = params(7);
    % y_cf = params(8);
    % z_cf = params(9);
    % phi_cf = params(10);
    % theta_cf = params(11);
    % psi_cf = params(12);

    X_vectors = data(:, 1:3)';
    X_cartesian = data(:, 4:6)';
    num_points = size(X_vectors, 2); % gets the number of points
    
    % make up I_tildeC
    C_vector = [0; y_f; z_f];
    I_tildeC = [eye(3), -C_vector];
    
    og_X_vectors = transformPointsToOriginal(X_vectors, params);

    % stores errors for each point and dimension
    errors = zeros(1, num_points);
    
    % rotation matrix
    R = rotationMatrixZYX(phi, theta, psi);
    K = createCameraMatrix(focal_length, y_f, z_f);
    
    % Compute error for each point
    for i = 1:num_points
        % 3D point onto image plane
        x_homogeneous = K * R * I_tildeC * [og_X_vectors(:, i); 1];
        
        % homogeneous to Cartesian
        x_cartesian = x_homogeneous ./ x_homogeneous(3);
        
        % squared error
        errors(i) = sum((x_cartesian - X_cartesian(:, i)).^2);
        % mae is sum(abs(x_cartesian - X_cartesian(:,i)));
    end
    % sum squared is sum(errors), mean absolute is mean(errors)
    mse = mean(errors);
end

% all_params = [phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf];
% camera_params = [focal_length, y_f, z_f];
function mae = computeMAError(params, cam_params, data)
    focal_length = cam_params(1);
    y_f = cam_params(2);
    z_f = cam_params(3);
    phi = params(1);
    theta = params(2);
    psi = params(3);
    % x_cf = params(7);
    % y_cf = params(8);
    % z_cf = params(9);
    % phi_cf = params(10);
    % theta_cf = params(11);
    % psi_cf = params(12);

    X_vectors = data(:, 1:3)';
    X_cartesian = data(:, 4:6)';
    num_points = size(X_vectors, 2); % gets the number of points
    
    % make up I_tildeC
    C_vector = [0; y_f; z_f];
    I_tildeC = [eye(3), -C_vector];
    
    og_X_vectors = transformPointsToOriginal(X_vectors, params);

    % stores errors for each point and dimension
    errors = zeros(1, num_points);
    
    % rotation matrix
    R = rotationMatrixZYX(phi, theta, psi);
    K = createCameraMatrix(focal_length, y_f, z_f);
    
    % Compute error for each point
    for i = 1:num_points
        % 3D point onto image plane
        x_homogeneous = K * R * I_tildeC * [og_X_vectors(:, i); 1];
        
        % homogeneous to Cartesian
        x_cartesian = x_homogeneous ./ x_homogeneous(3);
        
        % squared error
        errors(i) = sum(abs(x_cartesian - X_cartesian(:, i)));
        % mae is sum(abs(x_cartesian - X_cartesian(:,i)));
    end
    % sum squared is sum(errors), mean absolute is mean(errors)
    mae = mean(errors);
end

function med = computeMEDError(params, cam_params, data)
    focal_length = cam_params(1);
    y_f = cam_params(2);
    z_f = cam_params(3);
    phi = params(1);
    theta = params(2);
    psi = params(3);
    % x_cf = params(7);
    % y_cf = params(8);
    % z_cf = params(9);
    % phi_cf = params(10);
    % theta_cf = params(11);
    % psi_cf = params(12);

    X_vectors = data(:, 1:3)';
    X_cartesian = data(:, 4:6)';
    num_points = size(X_vectors, 2); % gets the number of points
    
    % make up I_tildeC
    C_vector = [0; y_f; z_f];
    I_tildeC = [eye(3), -C_vector];
    
    og_X_vectors = transformPointsToOriginal(X_vectors, params);

    % stores errors for each point and dimension
    errors = zeros(1, num_points);
    
    % rotation matrix
    R = rotationMatrixZYX(phi, theta, psi);
    K = createCameraMatrix(focal_length, y_f, z_f);
    
    % Compute error for each point
    for i = 1:num_points
        % 3D point onto image plane
        x_homogeneous = K * R * I_tildeC * [og_X_vectors(:, i); 1];
        
        % homogeneous to Cartesian
        x_cartesian = x_homogeneous ./ x_homogeneous(3);
        
        % squared error
        % errors(i) = sum(abs(x_cartesian - X_cartesian(:, i)));
        errors(i) = sqrt((x_cartesian(1) - X_cartesian(1,i))^2 + (X_cartesian(2) - X_cartesian(2,i))^2);

        % mae is sum(abs(x_cartesian - X_cartesian(:,i)));
    end
    % sum squared is sum(errors), mean absolute is mean(errors)
    med = mean(errors);
end

% function alpha_mae = AlphaTrimmedMeanAbsoluteError(params, calibration_params, data, alpha)
%     f_x = calibration_params(1);
%     f_y = calibration_params(2); %xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
%     p_x = calibration_params(3);
%     p_y = calibration_params(4);
%     x_cam = params(1);
%     y_cam = params(2);
%     z_cam = params(3);
%     phi_cam = params(4);
%     theta_cam = params(5);
%     psi_cam = params(6);
% 
%     cf_data = data(:, 1:3)';
%     pel_data = data(:, 4:5)';
% 
%     % num_points = size(cf_data, 2); % number of points
% 
% 
%     R = rotationMatrixZYX(phi_cam, theta_cam, psi_cam);
%     K = createCameraMatrix(f_x, f_y, p_x, p_y);
%     C_tilde = [x_cam; y_cam; z_cam];
% 
%     t = -R * C_tilde;
% 
%     % transforms cf data from cf system to world coord system using params
%     % 11 onwards, should be no difference though bc i defined cf and 
%     % world origins as the same
%     % world_cf_data = transformPointsToWorld(cf_data, params);
%     label = (pel_data(1,:) ~= 0);
%     index = find(label == 1);
% 
%     errors_matrix = zeros(1, length(index)); % stores errors
% 
%     for n = 1:length(index)
%         i = index(n);
%         proj_cf_data_homogen = K * [R, t] * [cf_data(:, i); 1];
%         proj_cf_data = proj_cf_data_homogen ./ proj_cf_data_homogen(3);
% 
%         % errors_matrix(i) = sum(abs(proj_cf_data(1:2) - pel_data(:, i)));
%         errors_matrix(n) = sqrt((proj_cf_data(1) - pel_data(1,i))^2 + (proj_cf_data(2) - pel_data(2,i))^2);
%     end
% 
%     % Apply alpha-trimmed mean
%     sorted_errors = sort(errors_matrix);
%     trimmed_errors = sorted_errors(floor(alpha*length(index))+1 : end-floor(alpha*length(index)));
%     alpha_mae = mean(trimmed_errors);
%     % mae = mean(errors_matrix);
% end