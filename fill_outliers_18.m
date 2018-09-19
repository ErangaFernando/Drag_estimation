clear all;
close all

filename = 'quadflight20180908_2speeds.mat'
load(filename)

DWbeacons = [data.DW_data.DW1 data.DW_data.DW2 data.DW_data.DW3 data.DW_data.DW4]
for i = 1:4
        B = filloutliers(DWbeacons(i).range(:,1),'linear','movmedian',15);
        DWbeacons(i).range(:,1) = B; 
end

figure
plot(data.DW_data.DW1.range(:,1))
hold on
plot(DWbeacons(1).range(:,1))

figure
plot(data.DW_data.DW2.range(:,1))
hold on
plot(DWbeacons(2).range(:,1))

figure
plot(data.DW_data.DW3.range(:,1))
hold on
plot(DWbeacons(3).range(:,1))

figure
plot(data.DW_data.DW4.range(:,1))
hold on
plot(DWbeacons(4).range(:,1))


 data.DW_data.DW1.range(:,1) = DWbeacons(1).range(:,1)
 data.DW_data.DW2.range(:,1) = DWbeacons(2).range(:,1)
 data.DW_data.DW3.range(:,1) = DWbeacons(3).range(:,1)
 data.DW_data.DW4.range(:,1) = DWbeacons(4).range(:,1)


