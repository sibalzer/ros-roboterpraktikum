# IO
set datafile separator ','
#set terminal png size 1800,1000
#set output 'coordinates-test.png'

# styling
set title 'Robot path from odometry'
#set label 'Path'
#show label
set xlabel 'X (m)'
set ylabel 'Y (m)'
#set style line 1 lw 3 lt rgb "#f62aa0" pt 2

plot 'odom-pose-test.csv' using 1:4 with lines lw 1 lt rgb "#f62aa0" title "Odometry", \
     'amcl-pose-test.csv' using 1:4 with lines lw 1 lt rgb "#0066ff" title "AMCL"
