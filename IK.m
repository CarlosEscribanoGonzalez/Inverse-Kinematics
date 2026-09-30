%% Cálculo de posiciones:
function joints = calculate_joints(skeleton, rots)
    %El primer joint se calcula aparte al ser el padre de la jerarquía
    joints = cell(1, size(skeleton, 1));
    rot0 = [1 0 0;  0 1 0; 0 0 1 ];
    trans0 = [1 0 skeleton(1,1);
                0 1 skeleton(1,2);
                0 0 1]; 
    joints{1} = rot0 * trans0;
    %Los siguientes joints utilizan los calculados anteriormente
    for i = 2:length(skeleton)
        rot = [ +cosd(rots(i)) -sind(rots(i)) 0;  
                    +sind(rots(i)) +cosd(rots(i)) 0; 
                    0           0            1 ];
        trans = [1 0 skeleton(i,1);
                     0 1 skeleton(i,2);
                     0 0 1]; 
        joints{i} = joints{i - 1} * rot * trans;
    end    
end

%% Dibujo del esqueleto:
function plot_skeleton(joints)
    scatter(joints{1}(1, 3), joints{1}(2, 3),450, 'o','filled','r');
    for i = 2:length(joints)
        line([joints{i}(1,3),joints{i-1}(1,3)],...
         [joints{i}(2,3),joints{i-1}(2,3)], 'LineWidth', 5);
        scatter(joints{i}(1,3), joints{i}(2,3),250, 'o','filled', ...
            'MarkerFaceColor',[.50 .8 .1]);
    end    
end

%% Matriz jacobiana:
function J = compute_jacobian(current_joints, skeleton, rots)
    angleVariation = 0.001;
    endEffectorPos = current_joints{length(current_joints)}(1:2, 3);
    J = zeros(2, length(rots));
    for i = 1:length(rots)
        %Se realiza variación de angleVariation en cada ángulo:
        rots(i) = rots(i) + angleVariation;
        %Se calcula la variación de posición correspondiente:
        variatedJoints = calculate_joints(skeleton, rots);
        endEffectorVariatedPos = variatedJoints{length(variatedJoints)}(1:2, 3);
        %Se añade el valor a la matriz jacobiana:
        J(:, i) = (endEffectorVariatedPos - endEffectorPos) / angleVariation;
        %Se deshace la variación de la rotación:
        rots(i) = rots(i) - angleVariation;
    end
end

%% Main:
clear; 
clc;
close all;

% defines joint positions in rest pose
trans0 = [0,0];
trans1 = [1,0]; % Offset with respect joint 0 in rest pose
trans2 = [2,1]; % Offset with respect joint 1 in rest pose
trans3 = [0,1]; % Offset with respect joint 2 in rest pose
   
rot0 = 0;
rot1 = 30;       % Local rotation for first joint
rot2 = -90;       % Local rotation for second joint
rot3 = 45;       % Local rotation for third joint
   
% puts skeleton defintion in vector
skeleton = [trans0; trans1; trans2; trans3];
rots = [rot0 rot1 rot2 rot3];

%Cálculo aleatorio de destino:
totalLength = 0;
for i = 1:length(skeleton)
    totalLength = totalLength + norm(skeleton(i, :));
end
target = rand(2, 1) * totalLength * 2 - totalLength;
%Se comprueba que no esté fuera de alcance ni muy cercano al centro
while norm(target) > totalLength || norm(target(:, 1)) < 0.2
    target = rand(2, 1) * totalLength * 2 - totalLength;
end
%Posición actual:
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
    % Calcula Jacobiano del estado actual
    J = compute_jacobian(current_joints, skeleton, rots);
    J_inv = pinv(J);
    % Calcula posición actual del end effector
    current_position = current_joints{length(current_joints)}(1:2, 3);
    % Calcula incrementos de las rotaciones
    angleInc = J_inv * (target - current_position);
    % Actualiza rotaciones
    for i = 1:length(rots)
        rots(i) = rots(i) + lr * angleInc(i);
    end
    % Dibuja el estado actual
    current_joints = calculate_joints(skeleton, rots);
    if clearFigure 
        cla;
    end
    scatter(target(1, 1), target(2, 1),450, 'o','filled','b'); %Target
    plot_skeleton(current_joints);
    drawnow;
end
    
hold off;