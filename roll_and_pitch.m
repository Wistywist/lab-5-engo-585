function [r , p] = roll_and_pitch(imu)
    r = atand(imu(:,2)./imu(:,3));

    p = atand(-imu(:,1) ./ ( imu(:,2).^2 + imu(:,3).^2).^0.5);
end