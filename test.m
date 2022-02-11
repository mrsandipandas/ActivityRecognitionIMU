imageDir = fullfile(toolboxdir('vision'), 'visiondata','upToScaleReconstructionImages');
images = imageDatastore(imageDir);
I1 = readimage(images, 1);
I2 = readimage(images, 2);

% Load precomputed camera parameters
load upToScaleReconstructionCameraParameters.mat

I1 = undistortImage(I1, cameraParams);
I2 = undistortImage(I2, cameraParams);

figure; imshow(I1); hold on
tic
imagePoints1 = detectFASTFeatures(rgb2gray(I1));
img_corners = insertMarker(I1,imagePoints1.selectStrongest(100).Location, '+','color','green','size',10);
toc
imshow(img_corners);

% Create the point tracker
tic
tracker = vision.PointTracker('MaxBidirectionalError', 3, 'NumPyramidLevels', 5);

% Initialize the point tracker
initialize(tracker, imagePoints1.Location, I1);

% Track the points
[imagePoints2, validIdx] = step(tracker, I2);
matchedPoints1 = imagePoints1.Location(validIdx, :);
matchedPoints2 = imagePoints2(validIdx, :);

% tform =  estimateGeometricTransform(matchedPoints2,matchedPoints1,'similarity')
tform = fitgeotrans(matchedPoints2,matchedPoints1,'nonreflectivesimilarity')
toc
% Visualize correspondences
figure
showMatchedFeatures(I1, I2, matchedPoints1, matchedPoints2);

title('Tracked Features');