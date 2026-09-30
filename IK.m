%% Joint position computation:
function joints = calculate_joints(skeleton, rots)
    % The first joint is computed separately since it is the root of the hierarchy
    joints = cell(1, size(skeleton, 1));
    rot0 = [1 0 0;  0 1 0; 0 0 1 ];
    trans0 = [1 0 skeleton(1,1);
              0 1 skeleton(1,2);
              0 0 1];
    joints{1} = rot0 * trans0;
    % The following joints use the previously computed ones
    for i = 2:length(skeleton)
        rot = [ +cosd(rots(i)) -sind(rots(i)) 0;
                +sind(rots(i)) +cosd(rots(i)) 0;
                0              0              1 ];
        trans = [1 0 skeleton(i,1);
                 0 1 skeleton(i,2);
                 0 0 1];
        joints{i} = joints{i - 1} * rot * trans;
    end
end

%% Skeleton drawing:
function plot_skeleton(joints)
    scatter(joints{1}(1, 3), joints{1}(2, 3),450, 'o','filled','r');
    for i = 2:length(joints)
        line([joints{i}(1,3),joints{i-1}(1,3)],...
             [joints{i}(2,3),joints{i-1}(2,3)], 'LineWidth', 5);
        scatter(joints{i}(1,3), joints{i}(2,3),250, 'o','filled', ...
                'MarkerFaceColor',[.50 .8 .1]);
    end
end

%% Jacobian matrix:
function J = compute_jacobian(current_joints, skeleton, rots)
    angleVariation = 0.001;
    endEffectorPos = current_joints{length(current_joints)}(1:2, 3);
    J = zeros(2, length(rots));
    for i = 1:length(rots)
        % Apply a small variation (angleVariation) to each angle:
        rots(i) = rots(i) + angleVariation;
        % Compute the corresponding change in end effector position:
        variatedJoints = calculate_joints(skeleton, rots);
        endEffectorVariatedPos = variatedJoints{length(variatedJoints)}(1:2, 3);
        % Store the finite-difference value as a column of the Jacobian:
        J(:, i) = (endEffectorVariatedPos - endEffectorPos) / angleVariation;
        % Undo the rotation variation:
        rots(i) = rots(i) - angleVariation;
    end
end

%% Main:
clear;
clc;
close all;

% defines joint positions in rest pose
trans0 = [0,0];
trans1 = [1,0];  % Offset with respect joint 0 in rest pose
trans2 = [2,1];  % Offset with respect joint 1 in rest pose
trans3 = [0,1];  % Offset with respect joint 2 in rest pose
rot0 = 0;
rot1 = 30;       % Local rotation for first joint
rot2 = -90;      % Local rotation for second joint
rot3 = 45;       % Local rotation for third joint

% puts skeleton defintion in vector
skeleton = [trans0; trans1; trans2; trans3];
rots = [rot0 rot1 rot2 rot3];

% Random target computation:
totalLength = 0;
for i = 1:length(skeleton)
    totalLength = totalLength + norm(skeleton(i, :));
end
target = rand(2, 1) * totalLength * 2 - totalLength;
% Check that the target is neither out of reach nor too close to the origin
while norm(target) > totalLength || norm(target(:, 1)) < 0.2
    target = rand(2, 1) * totalLength * 2 - totalLength;
end

% Current position:
current_joints = calculate_joints(skeleton, rots);
current_position = current_joints{length(current_joints)}(1:2, 3);

figure;
hold on;
grid on;
axis equal;
xlim([-4 4]);
ylim([-4 4]);

lr = 0.1;
maxError = 0.05;
clearFigure = true;

while (abs(norm(current_position - target)) > maxError)
    % Compute the Jacobian of the current state
    J = compute_jacobian(current_joints, skeleton, rots);
    J_inv = pinv(J);
    % Compute the current end effector position
    current_position = current_joints{length(current_joints)}(1:2, 3);
    % Compute the angle increments
    angleInc = J_inv * (target - current_position);
    % Update the rotations
    for i = 1:length(rots)
        rots(i) = rots(i) + lr * angleInc(i);
    end
    % Draw the current state
    current_joints = calculate_joints(skeleton, rots);
    if clearFigure
        cla;
    end
    scatter(target(1, 1), target(2, 1),450, 'o','filled','b'); % Target
    plot_skeleton(current_joints);
    drawnow;
end
hold off;