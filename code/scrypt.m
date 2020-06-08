clc;
clear;
format long;

% fileID = fopen('output.txt','w');

A = [
    0.921, 0, 0.041, 0;
    0, 0.918, 0, 0.033;
    0, 0, 0.924, 0;
    0, 0, 0, 0.937
];
 B = [
    0.017, 0.001;
    0.001, 0.023;
    0, 0.001;
    0.072, 0;
];
 C = [
    1,0,0,0; 
    0,1,0,0
];
 D = zeros(2, 2);
 N = 50;
 uSteady = [1; 1];
 ySteady = [0.65; 0.77];
 L = 30;
 n = 4;
 m = 2;
 p = 2;
 R = 10^-4 * eye(2);
 Q = 3 * eye(2);
 normValue = @(x, matrix) sqrt(x * matrix * x');
 l = @(u, y) normValue(u, R)^2 + normValue(y, Q)^2;

objective = @(x) objectiveFunc(x, n, L, l, uSteady, ySteady);

% {u^d, y^d}.
uData = zeros(1, N * 2);
yData = zeros(1, N * 2);
curX = zeros(n, 1);
for i = 1:N
    uData(1, [i * 2 - 1 i * 2]) = [ - 1 + 2 / N * (i - 1)];
    %uData(1, [i * 2 - 1 i * 2]) = [-1 + rand * 2 -1 + rand * 2];
    %yData([i * 2 - 1 i * 2]) = [(C * curX)'];
    %curX = A * curX + B * uData(1, [i * 2 - 1 i * 2])';
end
%figure(1);
%plot(uData(1:2:800), uData(2:2:800));
%figure(2);
plot(yData);

uRes = zeros(N + n, 2);
yRes = zeros(N + n, 2);

Aeq = zeros((L + n) * 4, (L + n) * 4 + N - (L + n) + 1);
beq = zeros(1, (L + n) * 4);
uHankel = hankelMatrix(reshape(uData, 2, N)', L + n, N);
yHankel = hankelMatrix(reshape(yData, 2, N)', L + n, N);
alphaShift = (L + n) * 4;
alphaRange = alphaShift + 1:(L + n) * 4 + N - (L + n) + 1;
for i = 1:4:(L + n) * 4
    Aeq(i, i) = 1;
    Aeq(i + 1, i + 1) = 1;
    Aeq(i + 2, i + 2) = 1;
    Aeq(i + 3, i + 3) = 1;
    hIndex = fix(i / 4) + 1;
    Aeq(i, alphaRange) = -uHankel(hIndex, :, 1);
    Aeq(i + 1, alphaRange) = -uHankel(hIndex, :, 2);
    Aeq(i + 2, alphaRange) = -yHankel(hIndex, :, 1);
    Aeq(i + 3, alphaRange) = -yHankel(hIndex, :, 2);
end
    
disp('MPC loop started.');
for t = 0:N - 1
    disp('iteration');
    disp(t);
    
    % Main constraint.
    timeIndex = n + 1 + t;
    Aeq(1:n * 4, alphaRange) = zeros(n * 4, N - (L + n) + 1);
    j = timeIndex - n;
    for i = 1:4:n * 4
        beq(i) = uRes(j, 1);
        beq(i + 1) = uRes(j, 2);
        beq(i + 2) = yRes(j, 1);
        beq(i + 3) = yRes(j, 2);
        j = j + 1;
    end
    
    % Initial constraint.
%     for i = n * 4 + 1:4:L * 4
%         beq(i) = uRes(j, 1);
%         beq(i + 1) = uRes(j, 2);
%         beq(i + 2) = yRes(j, 1);
%         beq(i + 3) = yRes(j, 2);
%         j = j + 1;
%     end
    
    % Terminal constraint.
    for i = (L + n) * 4 - n * 4 + 1:4:(L + n) * 4
        beq(i) = uSteady(1, 1);
        beq(i + 1) = uSteady(2, 1);
        beq(i + 2) = ySteady(1, 1);
        beq(i + 3) = ySteady(2, 1);
    end
    
    startedValues = [ ...
        beq(1:4 * n) ...
        zeros(1, (L + n) * 4 - 4 * n - 4 * n) ...
        beq((L + n) * 4 - n * 4 + 1:(L + n) * 4) ...
        zeros(1, N - (L + n) + 1)
    ];
    
    res = fmincon(objective, startedValues, [], [], Aeq, beq');
    res = res(1:(L + n) * 4);
    j = timeIndex - n;
    for i = 1:4:(L + n) * 4
        uRes(j, 1) = res(i);
        uRes(j, 2) = res(i + 1);
        yRes(j, 1) = res(i + 2);
        yRes(j, 2) = res(i + 3);
        j = j + 1;
    end
%     j = timeIndex;
%     i = n * 4 + 1;
%     uRes(j, 1) = res(i);
%     uRes(j, 2) = res(i + 1);
%     yRes(j, 1) = res(i + 2);
%     yRes(j, 2) = res(i + 3);
    %disp(res);
end

%fclose(fileID);


