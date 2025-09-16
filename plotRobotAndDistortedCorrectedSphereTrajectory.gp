set title "Motion Distortion Correction Project"
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set grid
set key left top
splot "build/distortedSphereTrajectory.dat" using 1:2:3 title "Distorted Point Cloud" lc rgb "blue", \
      "build/correctedSphereTrajectory.dat" using 1:2:3 title "Corrected Point Cloud" lc rgb "red", \
      "build/extractedHelicalTrajectory.dat" using 1:2:3 title "Robot Motion" lc rgb "green", \
      "build/generatedSpherePoints.dat" using 1:2:3 title "True Point Cloud" lc rgb "black"
pause -1
