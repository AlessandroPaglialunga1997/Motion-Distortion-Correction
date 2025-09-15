set title "Generated vs Extracted Helical Trajectory"
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set grid
set key left top
splot "build/generatedSphereTrajectory.dat" using 1:2:3 title "Sphere Motion Distortion" lc rgb "blue"
pause -1
