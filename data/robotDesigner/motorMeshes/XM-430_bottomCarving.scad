n = 128;
m = 32;
clearance = 0.0001;
delta = 0.000001;

HoleInR = 0.00125 + clearance;
HoleOutR = 2 * HoleInR;
HoleInW = 0.003;
HoleOutW = 0.2;


for (p = [[0.008, -0.03525, 0.006],
    [0.008, -0.03525, -0.006],
    [-0.008, -0.03525, -0.006],
    [-0.008, -0.03525, 0.006]]) {

x = p[0];
y = p[1];
z = p[2];

translate([x, y, z]){
    rotate(a = 90, v = [1, 0, 0]) {
        cylinder(h = HoleInW, r1 = HoleInR, r2 = HoleInR, center     = true, $fn = m);
    }
}

translate([x, y - 0.5 * (HoleInW + HoleOutW) + delta, z]){
    rotate(a = 90, v = [1, 0, 0]) {
        cylinder(h = HoleOutW, r1 = HoleOutR, r2 = HoleOutR, center = true, $fn = m);   
    }  
}

}





