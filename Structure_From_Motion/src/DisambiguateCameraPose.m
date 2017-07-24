function [C, R, X0] = DisambiguateCameraPose(Cs, Rs, Xset)
max_num = 0;
max_id = 0;
N = size(Xset{1}, 1);

for i = 1 : 4
    count = 0;
    R_cur = Rs{i};
    C_cur = Cs{i};
    X_cur = Xset{i};
    
    for k = 1 : N
        % physical point cannot locate behind two cameras
        % two cameras need to satisfy the above requirement at the same time 
        if (R_cur(3, :) * (X_cur(k, :)' - C_cur)) > 0 && (X_cur(k, 3) > 0)
            count = count + 1;
        end
    end
    
    if count > max_num
        max_num = count;
        max_id = i;
    end
end
C = Cs{max_id};
R = Rs{max_id};
X0 = Xset{max_id};
end