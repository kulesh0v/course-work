function retval = objectiveFunc(x,n, L, l, uSteady,ySteady)
%    display("n");
%    display(n);
%    display("L");
%    display(L);
%    uValues = reshape(x(n * 4 + 1:n * 4 + L * 2), 2, L)';
%    yValues = reshape(x(n * 4 + L * 2 + 1:n * 4 + L * 4), 2, L)';
%    disp(sum(arrayfun(@(k) l(uValues(k, :), yValues(k, :)), 1:L)));
%    retval = sum(arrayfun(@(k) l(uValues(k, :), yValues(k, :)), 1:L));
    uValues = zeros(L, 2);
    yValues = zeros(L, 2);
    j = 1;  
    for i = n * 4 + 1:4:(n + L) * 4
        uValues(j, 1) = x(i);
        uValues(j, 2) = x(i + 1);
        yValues(j, 1) = x(i + 2);
        yValues(j, 2) = x(i + 3);
        j = j + 1;
    end
    retval = sum(arrayfun(@(k) l(uValues(k, :) - uSteady', ...
                 yValues(k, :) - ySteady'), 1:L));
%    str = evalc('display("value:")');
%    fprintf(fileID, '%s\n', str);
%    str = evalc('display(retval)');
%    fprintf(fileID, '%s\n', str);
%    str = evalc('display(x)');
%    fprintf(fileID, '%s\n', str);
end