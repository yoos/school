#set title ""
set xlabel "x"
set ylabel "n"
set autoscale
set term png
set key left top
set output "collatz.png"
plot "collatz.t" using 1:2

