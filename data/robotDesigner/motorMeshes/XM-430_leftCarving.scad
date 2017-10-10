n = 128;
m = 32;
clearance = 0.0001;
delta = 0.000001;

HoleInR = 0.00125 + clearance;
HoleOutR = 2 * HoleInR;
HoleInW = 0.003;
HoleOutW = 0.2;


x3 = -0.01425;
y3 = -0.028;
z3 = 0.006;
translate([x3, y3, z3]){
    rotate(a = 90, v = [0, 1, 0]) {
        cylinder(h = HoleInW, r1 = HoleInR, r2 = HoleInR, center     = true, $fn = m);
    }
}

translate([x3 - 0.5 * (HoleInW + HoleOutW) + delta, y3, z3]){
    rotate(a = 90, v = [0, 1, 0]) {
        cylinder(h = HoleOutW, r1 = HoleOutR, r2 = HoleOutR, center = true, $fn = m);   
    }  
}

x4 = -0.01425;
y4 = -0.028;
z4 = -0.006;
translate([x4, y4, z4]){
    rotate(a = 90, v = [0, 1, 0]) {
        cylinder(h = HoleInW, r1 = HoleInR, r2 = HoleInR, center     = true, $fn = m);
    }
}

translate([x4 - 0.5 * (HoleInW + HoleOutW) + delta, y4, z4]){
    rotate(a = 90, v = [0, 1, 0]) {
        cylinder(h = HoleOutW, r1 = HoleOutR, r2 = HoleOutR, center = true, $fn = m);   
    }  
}


