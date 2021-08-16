# IO
set datafile separator ','
set terminal png size 1800,1000
set output 'pose-comparison.png'

# styling
set title 'Vergleich verschiedener Positionsanbieter'
#set label 'Path'
#show label
set xlabel 'X (m)'
set ylabel 'Y (m)'
#set style line 1 lw 3 lt rgb "#f62aa0" pt 2

plot 'pose-rec.csv' using 2:3 with lines lw 1 lt rgb "#" title "Vorgabe" \
     'pose-odom.csv' using 2:3 with lines lw 1 lt rgb "#f62aa0" title "Odometry", \
     'pose-amcl.csv' using 2:3 with lines lw 1 lt rgb "#0066ff" title "AMCL", \
     'pose-sim.csv' using 2:3 with lines lw 1 lt rgb "#ff9933" title "Simulator"
