#!/bin/bash
for f in MarkerData*.png
do
   convert $f -bordercolor green -border 300x300 border_$f
#  convert border_$f -flop border_$f
done
