function F = calc_error(X, measurement)
    ax = X(1);
    bx = X(2);
    ay = X(3);
    by = X(4);
    az = X(5);
    bz = X(6);

    %measurement = [acc_raw_X(1:1000) acc_raw_Y(1:1000) acc_raw_Z(1:1000) acc_calib_X(1:1000) acc_calib_Y(1:1000) acc_calib_Z(1:1000)];

    E_x = measurement(:,4) - (measurement(:,1)*ax + bx);
    E_y = measurement(:,5) - (measurement(:,2)*ay + by);
    E_z = measurement(:,6) - (measurement(:,3)*az + bz);

    F = [E_x; E_y; E_z]';
end