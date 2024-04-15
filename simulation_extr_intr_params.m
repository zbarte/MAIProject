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
all_params = [focal_length, y_f, z_f, phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf];

% Transforms points into cf system (in reality we would not know the
% transformation but would only know these measurements from the cf)
X_vectors = points';
points_in_cf = generateRandomPointsInCf(X_vectors, all_params);
points_in_cf_trans = points_in_cf';

% og_X_vectors = transformPointsToOriginal(points_in_cf_trans, all_params);

% checking transformation against old way of doing it
% X_vectors_M1 = points';


x_cartesian = projectPointsCartesianWithNoise(all_params, points_in_cf_trans, noise_std);  
% x_cartesian_M1 = projectPointsCartesianWithNoiseM1(all_params, X_vectors_M1, noise_std); 
data = [points_in_cf, x_cartesian'];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Optimisation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% same as testing all_params but testing how far off initial guess can be.
initial_params = [focal_length, y_f, z_f, phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf]; 
initial_params = initial_params + 20*rand(size(initial_params));

% compute mean squared error
mse = computeMSError(all_params, data);
fprintf('Mean squared error: %.2f\n', mse);

% compute mean absolute error
mae = computeMAError(all_params, data);
fprintf('Mean absolute error: %.2f\n', mae);

% compute mean euclidean distance error
med = computeMEDError(all_params, data);
fprintf('Mean Euclidean distance error: %.2f\n', med);

options = optimset('MaxFunEvals', 10000);  % Increase the limit to 10000
optimised_MSE_params = fminsearch(@(params) computeMSError(params, data), all_params);
optimised_MAE_params = fminsearch(@(params) computeMAError(params, data), all_params);
optimised_MED_params = fminsearch(@(params) computeMEDError(params, data), all_params, options);

fprintf('Initial parameters: \n');
fprintf('%.2f ', initial_params);
fprintf('\n');
fprintf('\nOptimal MSE parameters: \n');
fprintf('%.2f ', optimised_MSE_params);
fprintf('\n');
fprintf('\nOptimal MAE parameters: \n');
fprintf('%.2f ', optimised_MAE_params);
fprintf('\n');
fprintf('\nOptimal Med parameters: \n');
fprintf('%.2f ', optimised_MED_params);
fprintf('\n');

% compute optimised mean squared error
mse_optimised = computeMSError(optimised_MSE_params, data);
fprintf('Optimal mean squared error: %.2f\n', mse_optimised);

% compute optimised mean absolute error
mae_optimised = computeMAError(optimised_MAE_params, data);
fprintf('Optimal mean absolute error: %.2f\n', mae_optimised);

% compute optimised mean absolute error
med_optimised = computeMEDError(optimised_MED_params, data);
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


function points_in_cf = generateRandomPointsInCf(points, all_params)
    % Extract CF parameters
    % x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf
    x_cf = all_params(7);
    y_cf = all_params(8);
    z_cf = all_params(9);
    phi_cf = all_params(10);
    theta_cf = all_params(11);
    psi_cf = all_params(12);
    
    % random points in original coordinate system
    % points = generateRandomPointsWithNoise(num_points, noise_std);
    
    % rotation matrix in cf system
    R_cf = rotationMatrixZYX(phi_cf, theta_cf, psi_cf);

    % Apply rotation to points
    points_rotated = (R_cf * points)';

    % Translate points to CF coordinate system
    points_in_cf = points_rotated + [x_cf, y_cf, z_cf];
end

% THIS IS FOR METHOD 1
function og_X_vectors = transformPointsToOriginal(X_vectors_cf, params)
    % Extract CF parameters
    x_cf = params(7);
    y_cf = params(8);
    z_cf = params(9);
    phi_cf = params(10);
    theta_cf = params(11);
    psi_cf = params(12);
    
    % Calculate rotation matrix from CF to original coordinate system
    R_cf = rotationMatrixZYX(phi_cf, theta_cf, psi_cf);
    
    % Calculate inverse of the rotation matrix
    R_cf_inv = inv(R_cf);
    
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
% [focal_length, y_f, z_f, phi, theta, psi, x_cf, y_cf, z_cf, phi_cf, theta_cf, psi_cf];
function x_cartesian = projectPointsCartesianWithNoise(params, X_vectors, noise_std)    
    num_points = size(X_vectors, 2); 
    
    focal_length = params(1);
    y_f = params(2);
    z_f = params(3);
    phi = params(4);
    theta = params(5);
    psi = params(6);
    x_cf = params(7);
    y_cf = params(8);
    z_cf = params(9);
    phi_cf = params(10);
    theta_cf = params(11);
    psi_cf = params(12);

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
end % FIX THIS ONE !!!!! - trying to convert cf points back into world points 
% and then projecting them

% define a function that does what x_cartesian does but without doing the
% weird cf transformation to see if they're the same - should mostly be the
% same function
% THIS IS FOR METHOD 1
function x_cartesian_M1 = projectPointsCartesianWithNoiseM1(params, X_vectors, noise_std)
    num_points = size(X_vectors, 2); 
    
    focal_length = params(1);
    y_f = params(2);
    z_f = params(3);
    phi = params(4);
    theta = params(5);
    psi = params(6);

    % stores cartesian coordinates of projected points
    x_cartesian_M1 = zeros(3, num_points);
    
    % calculates rotation matrix
    R = rotationMatrixZYX(phi, theta, psi);
    K = createCameraMatrix(focal_length, y_f, z_f);
    % make up I_tildeC
    C_vector = [0; y_f; z_f];
    I_tildeC = [eye(3), -C_vector];

        % Project each point onto the image plane
    for i = 1:num_points
        % Project 3D point onto image plane
        x_homogeneous = K * R * I_tildeC * [X_vectors(:, i); 1];
        
        x_cartesian_M1(:, i) = x_homogeneous ./ x_homogeneous(3); % homogeneous to cartesian
        
        noise = noise_std * randn(3, 1); % adds noise
        x_cartesian_M1(:, i) = x_cartesian_M1(:, i) + noise;
    end

end

function mse = computeMSError(params, data)
    focal_length = params(1);
    y_f = params(2);
    z_f = params(3);
    phi = params(4);
    theta = params(5);
    psi = params(6);
    x_cf = params(7);
    y_cf = params(8);
    z_cf = params(9);
    phi_cf = params(10);
    theta_cf = params(11);
    psi_cf = params(12);

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

function mae = computeMAError(params, data)
    focal_length = params(1);
    y_f = params(2);
    z_f = params(3);
    phi = params(4);
    theta = params(5);
    psi = params(6);
    x_cf = params(7);
    y_cf = params(8);
    z_cf = params(9);
    phi_cf = params(10);
    theta_cf = params(11);
    psi_cf = params(12);

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

function med = computeMEDError(params, data)
    focal_length = params(1);
    y_f = params(2);
    z_f = params(3);
    phi = params(4);
    theta = params(5);
    psi = params(6);
    x_cf = params(7);
    y_cf = params(8);
    z_cf = params(9);
    phi_cf = params(10);
    theta_cf = params(11);
    psi_cf = params(12);

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
