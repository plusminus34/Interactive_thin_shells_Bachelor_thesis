n = 128;
m = 32;
clearance = 0.0001;
delta = 0.000001;
HornOutR = 0.00975 + clearance;
HornInR = 0.0045 + clearance;
HornOutW = 0.002;
HornInW = 0.005;
HoleInR = 0.001 + clearance;
HoleOutR = 2 * HoleInR;
HoleInW = 0.003;
HoleOutW = 0.006;

sizeX = 0.0285;
sizeY = 0.0465;
sizeZ = 0.034;

translate([0, -0.012, 0]){
    cube(size = [sizeX, sizeY, sizeZ], center = true);
}


// LEFT
translate([0, 0, -0.017 - 0.5 * HornOutW]){
    cylinder(h = HornOutW, r1 = HornOutR, r2 = HornOutR, center     = true, $fn = n);
}

translate([0, -HornOutR, -0.017 - 0.5 * HornOutW]){
    cube(size = [HornOutR * 3, HornOutR * 2, HornOutW], center = true);
}

translate([0, 0, -0.019 - 0.5 * HornInW + delta]){
    cylinder(h = HornInW, r1 = HornInR, r2 = HornInR, center     = true, $fn = n);
}

translate([0, -0.5 * HornOutR, -0.019 - 0.5 * HornInW + delta]){
    cube(size = [HornInR * 2, HornOutR, HornInW], center = true);
}

translate([0, -HornOutR - 0.004, -0.019 - 0.5 * HornInW + delta]){
    cube(size = [HornOutR * 3, HornOutR * 2, HornInW], center = true);
}

translate([0, 0, -0.017 - 0.5 * HornOutW]){
    cylinder(h = HornOutW, r1 = HornOutR, r2 = HornOutR, center     = true, $fn = n);
}

for (i = [0:4]) {
    x = 0.008 * cos(45 * i);
    y = 0.008 * sin(45 * i);
    translate([x, y, -0.019]){
        cylinder(h = HoleInW, r1 = HoleInR, r2 = HoleInR, center     = true, $fn = m);
    }

    translate([x, y, -0.019 - 0.5 * (HoleInW + HoleOutW) + delta]){
        cylinder(h = HoleOutW, r1 = HoleOutR, r2 = HoleOutR, center     = true, $fn = m);
    }

}


// RIGHT
translate([0, 0, 0.017 + 0.5 * HornOutW]){
    cylinder(h = HornOutW, r1 = HornOutR, r2 = HornOutR, center     = true, $fn = n);
}

translate([0, -HornOutR, 0.017 + 0.5 * HornOutW]){
    cube(size = [HornOutR * 3, HornOutR * 2, HornOutW], center = true);
}

translate([0, 0, 0.019 + 0.5 * HornInW - delta]){
    cylinder(h = HornInW, r1 = HornInR, r2 = HornInR, center     = true, $fn = n);
}

translate([0, -0.5 * HornOutR, 0.019 + 0.5 * HornInW - delta]){
    cube(size = [HornInR * 2, HornOutR, HornInW], center = true);
}

translate([0, -HornOutR - 0.004, 0.019 + 0.5 * HornInW - delta]){
    cube(size = [HornOutR * 3, HornOutR * 2, HornInW], center = true);
}

for (i = [0:4]) {
    x = 0.008 * cos(45 * i);
    y = 0.008 * sin(45 * i);
    translate([x, y, 0.019]){
        cylinder(h = HoleInW, r1 = HoleInR, r2 = HoleInR, center     = true, $fn = m);
    }

    translate([x, y, 0.019 + 0.5 * (HoleInW + HoleOutW) - delta]){
        cylinder(h = HoleOutW, r1 = HoleOutR, r2 = HoleOutR, center     = true, $fn = m);
    }

}
