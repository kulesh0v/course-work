function [ y, curX ] = dynamicSystemFunc( u, x, A, B, C)
disp('x');
disp(x);
curX = A * x + B * u;
y = C * x;
disp('y');
disp(y);
end

