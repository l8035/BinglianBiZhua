clc
clear
syms xw yw u v m1 m2 m4 A B D F E H
eq1=xw*(m1*v-E)+yw*(m2*v-F)-H+m4*v;
eq2=xw*(m1*u-A)+yw*(m2*u-B)-D+m4*u;
sol = solve(eq1, eq2, xw, yw );
sol.xw
sol.yw

