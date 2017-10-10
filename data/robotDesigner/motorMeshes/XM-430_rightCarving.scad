n = 128;
m = 32;
clearance = 0.0001;
delta = 0.000001;

HoleInR = 0.00125 + clearance;
HoleOutR = 2 * HoleInR;
HoleInW = 0.003;
HoleOutW = 0.2;

x1 = 0.01425;
y1 = -0.028;
z1 = 0.006;
translate([x1, y1, z1]){
    rotate(a = 90, v = [0, 1, 0]) {
        cylinder(h = HoleInW, r1 = HoleInR, r2 = HoleInR, center     = true, $fn = m);
    }
}

translate([x1 + 0.5 * (HoleInW + HoleOutW) - delta, y1 , z1]){
    rotate(a = 90, v = [0, 1, 0]) {
        cylinder(h = HoleOutW, r1 = HoleOutR, r2 = HoleOutR, center = true, $fn = m);   
    }  
}


x2 = 0.01425;
y2 = -0.028;
z2 = -0.006;
translate([x2, y2, z2]){
    rotate(a = 90, v = [0, 1, 0]) {
        cylinder(h = HoleInW, r1 = HoleInR, r2 = HoleInR, center     = true, $fn = m);
    }
}

translate([x2 + 0.5 * (HoleInW + HoleOutW) - delta, y2, z2]){
    rotate(a = 90, v = [0, 1, 0]) {
        cylinder(h = HoleOutW, r1 = HoleOutR, r2 = HoleOutR, center = true, $fn = m);   
    }  
}


