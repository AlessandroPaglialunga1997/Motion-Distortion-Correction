set title "Motion Distortion Correction Project"
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set grid
set key left top
splot "output_data/distortedSphereTrajectory.dat" using 1:2:3 title "Distorted Point Cloud" lc rgb "blue", \
      "output_data/correctedSphereTrajectory.dat" using 1:2:3 title "Corrected Point Cloud" lc rgb "red", \
      "output_data/extractedHelicalTrajectory.dat" using 1:2:3 title "Robot Motion" lc rgb "green", \
      "output_data/generatedSpherePoints.dat" using 1:2:3 title "True Point Cloud" lc rgb "black"
pause -1
