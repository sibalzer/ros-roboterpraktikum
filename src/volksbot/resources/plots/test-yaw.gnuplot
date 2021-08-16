# IO
set datafile separator ','
set terminal png size 1800,1000
set output 'test-yaw.png'

# styling
set title 'Yaw Vergleich von odom und AMCL'
#set label 'Path'
#show label
set xlabel 'System time (s)'
set ylabel 'Yaw (rad)'
#set style line 1 lw 3 lt rgb "#f62aa0" pt 2

plot 'test-yaw-pose-odom.csv' using 1:4 with lines lw 1 lt rgb "#f62aa0" title "Odometry", \
     'test-yaw-pose-amcl.csv' using 1:4 with lines lw 1 lt rgb "#0066ff" title "AMCL"
