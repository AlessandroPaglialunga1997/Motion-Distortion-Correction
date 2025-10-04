set title "True Robot Trajecotry VS Extracted Trajectory from IMU"
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set grid
set key left top
splot "output_data/quadHardTrueTrajectory.dat" using 1:2:3 title "Generated" lc rgb "blue"
pause -1
