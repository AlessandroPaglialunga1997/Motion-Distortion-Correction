set title "Generated vs Extracted Helical Trajectory"
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set grid
set key left top
splot "build/distortedSphereTrajectory.dat" using 1:2:3 title "Distorted" lc rgb "blue", \
      "build/correctedSphereTrajectory.dat" using 1:2:3 title "Corrected" lc rgb "red"
pause -1
