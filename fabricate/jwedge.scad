$fn=50;
// rails

mm = 1;
inch = 25.4 * mm;

POST_R = 35 * mm / 2.;
SCREW_R = 2.6 * mm / 2.;
SCREW_Y_SEP = 19 * mm;
SCREW_Z_SEP = 75 * mm;
SCREW_OFF = 5.5 * mm;
BAND_W = 20 * mm; // for .75" strap
BAND_W = 1*inch + 1 * mm; // for 1" strap
BAND_T = 3 * mm;
BAND_OFF = 6 * mm;

BORE_R = 6 * mm / 2;

Z = 83 * mm;
Z = 86 * mm;
X0 = 25 * mm;
X1 = 26.5 * mm;
X = X0 + X1;
Y = 27 * mm;
Y = 30 * mm;
POST_ANG = 18;

module jwedge(){
  intersection(){
    difference(){
      // main wedge
      difference(){
	cube([X, Y, Z]);
	rotate(a=POST_ANG, v=[0, 1, 0])
	  union(){
	  translate([-X, -1, 0]) cube([X, Y + 2, 2 * Z]);
	  translate([0, Y/2, 0]) cylinder(r=POST_R, h = 10 * Z + 2);
	}  
      }
      // screw holes
      translate([0, SCREW_OFF, SCREW_OFF])
	rotate(a=90, v=[0, 1, 0])cylinder(h=100*mm, r=SCREW_R);
      translate([0, SCREW_OFF + SCREW_Y_SEP, SCREW_OFF])
	rotate(a=90, v=[0, 1, 0])cylinder(h=100*mm, r=SCREW_R);
      translate([0, SCREW_OFF + SCREW_Y_SEP, SCREW_OFF + SCREW_Z_SEP])
	rotate(a=90, v=[0, 1, 0])cylinder(h=100*mm, r=SCREW_R);
      translate([0, SCREW_OFF, SCREW_OFF + SCREW_Z_SEP])
	rotate(a=90, v=[0, 1, 0])cylinder(h=100*mm, r=SCREW_R);
  
      // counter bores
      translate([0, SCREW_OFF, SCREW_OFF])
	rotate(a=90, v=[0, 1, 0])cylinder(h=X - 4*mm, r=BORE_R);
      translate([0, SCREW_OFF + SCREW_Y_SEP, SCREW_OFF])
	rotate(a=90, v=[0, 1, 0])cylinder(h=X - 4*mm, r=BORE_R);
      translate([0, SCREW_OFF + SCREW_Y_SEP, SCREW_OFF + SCREW_Z_SEP])
	rotate(a=90, v=[0, 1, 0])cylinder(h=X - 4*mm, r=BORE_R);
      translate([0, SCREW_OFF, SCREW_OFF + SCREW_Z_SEP])
	rotate(a=90, v=[0, 1, 0])cylinder(h=X - 4*mm, r=BORE_R);

      // BAND
      rotate(a=POST_ANG, v=[0, 1, 0])
	translate([0, Y/2, Z/3])
	difference(){
	cylinder(r=POST_R + BAND_OFF +  + BAND_T, h=BAND_W);
	translate([0, 0, -1])cylinder(r=POST_R + 6 * mm, h=BAND_W + 2);
    
      }
    }

    // round edges
    translate([0, 1.5*mm, 1.5*mm])
      minkowski(){
      cube([X, Y - 3 * mm, Z - 3 * mm]);
      rotate(v=[0,1, 0], a=90)cylinder(r=1.5*mm,h=10);
    }
  }
}
rotate(a=90, v=[0, 1, 0]) jwedge();
