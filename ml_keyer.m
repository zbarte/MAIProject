close all
clear

image = imread('exp2_s1_1956.tif'); % Load your image
imshow(image);

% convert image to grayscale
if size(image, 3) == 3
    grayImage = rgb2gray(image);
else
    grayImage = image;
end

objectMask = grayImage > 245; % fg threshold value
backgroundMask = grayImage <= 245; % bg threshold value

% Estimate parameters of the probability distributions
objMean = mean(grayImage(objectMask)); % Mean intensity of object pixels
objStd = std(double(grayImage(objectMask))); % Standard deviation of object pixels
bgMean = mean(grayImage(backgroundMask)); % Mean intensity of background pixels
bgStd = std(double(grayImage(backgroundMask))); % Standard deviation of background pixels

% likelihood of each pixel
likelihoodObj = normpdf(double(grayImage), objMean, objStd);
likelihoodBg = normpdf(double(grayImage), bgMean, bgStd);

% ml threshold
binaryMask = likelihoodObj > likelihoodBg;

% bounding box to exclude
top_left = [3577, 1];
bottom_right = [3840, 354];

% Exclude bounding box from binary mask
for i = top_left(2):bottom_right(2)
    for j = top_left(1):bottom_right(1)
        binaryMask(i, j) = 0;
    end
end

% Find connected components (blob) in the modified binary mask
cc = bwconncomp(binaryMask);

% centroid of blob
stats = regionprops(cc, 'Area', 'Centroid');

% finds the index of the largest blob
[~, idx] = max([stats.Area]);

% centroid coordinates of largest blob
centroid = stats(idx).Centroid;

% Display the modified binary mask
imshow(image);
hold on;

% Plot the centroid on top of the original image
plot(centroid(1), centroid(2), 'r+', 'MarkerSize', 10, 'LineWidth', 2);

% Save the superimposed binary mask
% imwrite(binaryMask, 'modified_binary_mask.jpg');

% Plot the PDF
% figure;
% plot(double(grayImage), likelihoodObj, 'LineWidth', 2);
% xlabel('x');
% ylabel('Probability Density');
% title('Normal Distribution PDF');
% grid on;