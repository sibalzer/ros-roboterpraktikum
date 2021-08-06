set datafile separator ','

# styling
set title 'Robot path from odometry'
#set label 'Path'
#show label
set xlabel 'X (m)'
set ylabel 'Y (m)'
#set style line 1 lw 3 lt rgb "#f62aa0" pt 2

plot 'coordinates.csv' using 1:2 with lines lw 1 lt rgb "#f62aa0" notitle