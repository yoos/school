#set title ""
set xlabel "x"
set ylabel "n"
set autoscale
set term png
set key off
set output "collatz.png"
set pointsize 0.2
plot "collatz.t" using 1:2 pt 3

