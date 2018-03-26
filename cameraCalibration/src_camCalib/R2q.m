function [ q ] = R2q( Rs )
%R2Q Converts to rotation to quaternion

q = zeros(4, 1, size(Rs, 3));

for i=1:size(Rs, 3)
    R = Rs(:, :, i);

    tr = trace(R);
    if tr>0
        s = 0.5/sqrt(tr+1);
        w = 0.25/s;
        x = (R(3, 2) - R(2, 3))*s;
        y = (R(1, 3) - R(3, 1))*s;
        z = (R(2, 1) - R(1, 2))*s;
    else
        if R(1, 1) > R(2, 2) && R(1, 1) > R(3, 3)
            s = 2*sqrt(1 + R(1, 1) - R(2, 2) - R(3, 3));
            w = (R(3, 2) - R(2, 3))/s;
            x = 0.25*s;
            y = (R(1, 2) + R(2, 1))/s;
            z = (R(1, 3) + R(3, 1))/s;
        elseif R(2, 2) > R(3, 3)
            s = 2*sqrt(1 + R(2, 2) - R(1, 1) - R(3, 3));
            w = (R(1, 3) - R(3, 1) ) / s;
            x = (R(1, 2) + R(2, 1) ) / s;
            y = 0.25 * s;
            z = (R(2, 3) + R(3, 2) ) / s;
        else
            s = 2 * sqrt( 1.0 + R(3, 3) - R(1, 1) - R(2, 2) );
            w = (R(2, 1) - R(1, 2) ) / s;
            x = (R(1, 3) + R(3, 1) ) / s;
            y = (R(2, 3) + R(3, 2) ) / s;
            z = 0.25 * s;
        end
    end
    q(:, 1, i) = [w; x; y; z];
end
end

