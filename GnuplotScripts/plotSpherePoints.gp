set title "Generated vs Extracted Helical Trajectory"
set xlabel "X"
set ylabel "Y"
set zlabel "Z"
set grid
set key left top
splot "output_data/generatedSpherePoints.dat" using 1:2:3 title "Generated" lc rgb "blue"
pause -1
