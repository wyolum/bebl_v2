$fn=50;
mm = 1;
inch = 25.4 * mm;

SCREW_R = 2.6 * mm / 2.;
SCREW_Y_SEP = 19 * mm;
SCREW_Z_SEP = 75 * mm;
SCREW_OFF = 5.5 * mm;
LED_R = 5.5 * mm / 2;
LED_H = 10 * mm;

BORE_R = 6 * mm / 2;

Z = 86 * mm;
X = 5*mm;
Y = 27 * mm;
Y = 30 * mm;
T = 1.5 * mm;

DEPTH = X - T;
POST_ANG = 18;


LED_OFF = 14.5 * mm;
LED_SEP = 10 * mm;
N_LED = 5;

module base(){
  difference(){
    cube([X, Y, Z]);
    
    // screw holes
    translate([-1, SCREW_OFF, SCREW_OFF])
      rotate(a=90, v=[0, 1, 0])cylinder(h=100*mm, r=SCREW_R);
    translate([-1, SCREW_OFF + SCREW_Y_SEP, SCREW_OFF])
      rotate(a=90, v=[0, 1, 0])cylinder(h=100*mm, r=SCREW_R);
    translate([-1, SCREW_OFF + SCREW_Y_SEP, SCREW_OFF + SCREW_Z_SEP])
      rotate(a=90, v=[0, 1, 0])cylinder(h=100*mm, r=SCREW_R);
    translate([-1, SCREW_OFF, SCREW_OFF + SCREW_Z_SEP])
      rotate(a=90, v=[0, 1, 0])cylinder(h=100*mm, r=SCREW_R);
    
    // counter bores
    translate([-1, SCREW_OFF, SCREW_OFF])
      rotate(a=90, v=[0, 1, 0])cylinder(h=X - 1*mm + 1, r=BORE_R);
    translate([-1, SCREW_OFF + SCREW_Y_SEP, SCREW_OFF])
      rotate(a=90, v=[0, 1, 0])cylinder(h=X - 1*mm + 1, r=BORE_R);
    translate([-1, SCREW_OFF + SCREW_Y_SEP, SCREW_OFF + SCREW_Z_SEP])
      rotate(a=90, v=[0, 1, 0])cylinder(h=X - 1*mm + 1, r=BORE_R);
    translate([-1, SCREW_OFF, SCREW_OFF + SCREW_Z_SEP])
      rotate(a=90, v=[0, 1, 0])cylinder(h=X - 1*mm + 1, r=BORE_R);
  }
}

module cover(){
  SCREW_PAD = 9 * mm;
  difference(){
    intersection(){
      base();
      // round edges
      translate([0, 1.5*mm, 1.5*mm])
	minkowski(){
	cube([X, Y - 3 * mm, Z - 3 * mm]);
	rotate(v=[0,1, 0], a=90)cylinder(r=1.5*mm,h=10);
      }
    }
    //hollow out
    translate([T, T, SCREW_PAD])cube([DEPTH + 1, Y - 2 * T, Z - 2 * SCREW_PAD]);
    translate([T, SCREW_PAD, T])cube([DEPTH + 1, Y - 2 * SCREW_PAD, Z - 2 * T]);
  }
}
difference(){
  cover();
  for(i = [0:N_LED-1]){
    translate([-1, Y/2, LED_OFF + i * LED_SEP])rotate(a=90, v=[0, 1, 0])cylinder(r=LED_R, h=LED_H);
  }
  translate([-1, Y/2, 73*mm])rotate(a=90, v=[0, 1, 0])cylinder(r=LED_R, h=LED_H);
}

