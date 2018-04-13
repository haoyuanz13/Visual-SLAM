function [Cs, Rs] = ExtractCameraPose(E)
%% Decompose E matrix (Essential matrix) to obtain R and t

[u, ~, v] = svd(E);
W = [0, -1, 0; 
     1, 0, 0; 
     0, 0, 1];

% ts is the set of translation
% Rs is the set of rotation
ts = cell(4, 1);
Rs = cell(4, 1);

% four configurations 
ts{1} = u(:, 3);
Rs{1} = u * W * v';

ts{2} = -u(:, 3);
Rs{2} = u * W * v';

ts{3} = u(:, 3);
Rs{3} = u * W' * v';

ts{4} = -u(:, 3);
Rs{4} = u * W' * v';

% Cs is the translation measured in first person view
Cs = cell(4, 1);
for i = 1 : 4
    t_cur = ts{i};
    R_cur = Rs{i};
    C_cur = -R_cur' * t_cur;
    Cs{i} = C_cur;
    
    if det(R_cur) < 0
        Cs{i} = -C_cur;
        Rs{i} = -R_cur;
    end
end    
end
