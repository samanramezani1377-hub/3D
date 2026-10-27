// Demo modular guarded quadcopter concept — dimensions in mm
// Parametric printable concept, NOT flight-certified. Export each part by setting PART.
$fn=96;
PART = "assembly"; // assembly, top, bottom, arm, rotor_guard, propeller, camera_mount
span=240; body_w=104; body_l=112; body_h=26; wall=2.4;
guard_od=72; guard_id=64; guard_h=18; arm_w=14; arm_h=8;
motor_d=16; motor_h=5; prop_d=58; prop_t=2.2;
screw_d=3.2; boss_d=8; camera_w=24; camera_h=18;
module rounded_box(size,r=5) { minkowski(){ cube([size[0]-2*r,size[1]-2*r,size[2]-2*r],center=true); cylinder(r=r,h=0.01); } }
module screw_holes(z0,z1) {
  for(x=[-body_w/2+8,body_w/2-8], y=[-body_l/2+8,body_l/2-8])
    translate([x,y,z0-1]) cylinder(d=screw_d,h=z1-z0+2);
}
module bottom_shell(){
 difference(){
  union(){
   translate([0,0,4]) rounded_box([body_w,body_l,8],7);
   // continuous flat internal floor and perimeter walls
   translate([0,0,body_h/2]) difference(){ rounded_box([body_w,body_l,body_h],7); translate([0,0,wall]) rounded_box([body_w-2*wall,body_l-2*wall,body_h],5); }
   for(x=[-body_w/2+8,body_w/2-8], y=[-body_l/2+8,body_l/2-8])
     translate([x,y,body_h/2]) cylinder(d=boss_d,h=body_h);
  }
  screw_holes(-1,body_h+2);
 }
}
module top_shell(){
 difference(){
  union(){
   translate([0,0,body_h+4]) rounded_box([body_w,body_l,8],7);
   // shallow top cover with underside lip
   translate([0,0,body_h+9]) difference(){ rounded_box([body_w,body_l,10],7); translate([0,0,-wall]) rounded_box([body_w-2*wall,body_l-2*wall,10],5); }
   for(x=[-body_w/2+8,body_w/2-8], y=[-body_l/2+8,body_l/2-8])
     translate([x,y,body_h+1]) cylinder(d=10,h=8);
  }
  screw_holes(body_h,body_h+15);
 }
}
module arm_one(){
 // Arm is sandwiched between top and bottom; holes for motor at outer end
 difference(){
  union(){
   linear_extrude(height=arm_h) polygon([[-arm_w/2,-8],[arm_w/2,-8],[arm_w/2,68],[guard_od/2+3,68],[guard_od/2+3,76],[-guard_od/2-3,76],[-guard_od/2-3,68],[-arm_w/2,68]]);
   translate([0,0,arm_h/2]) cube([body_w/2+5,arm_w,arm_h],center=false);
   translate([0,68,arm_h/2]) cylinder(d=guard_od+6,h=arm_h,center=true);
  }
  translate([0,0,-1]) cylinder(d=screw_d,h=arm_h+2);
  translate([0,68,-1]) cylinder(d=motor_d+1,h=arm_h+2);
  for(x=[-6,6]) translate([x,8,-1]) cylinder(d=screw_d,h=arm_h+2);
 }
}
module guard(){
 difference(){
  union(){ cylinder(d=guard_od,h=guard_h); translate([0,0,guard_h/2]) cylinder(d=guard_od+2,h=guard_h/2,center=true); }
  translate([0,0,-1]) cylinder(d=guard_id,h=guard_h+2);
 }
}
module propeller(){
 // visual prototype only: broad, mildly swept 3-blade rotor; replace with balanced engineered prop.
 // print a hub template only; do not fly a 3D-printed propeller without dynamic balancing and thrust testing.
 difference(){
  union(){
   cylinder(d=12,h=prop_t);
   for(a=[0:120:359]) rotate([0,0,a]) translate([8,0,0])
    hull(){ translate([0,0,0]) cylinder(d=4,h=prop_t); translate([prop_d/2-5,2,0]) cylinder(d=2.8,h=prop_t); }
  }
  translate([0,0,-1]) cylinder(d=3.2,h=prop_t+2);
 }
}
module camera_mount(){
 // fixed downward-tilted camera plate; angle set by wedge geometry
 difference(){
  hull(){ translate([0,0,4]) cube([camera_w,camera_h,3],center=true); translate([0,-4,10]) cube([camera_w,8,3],center=true); }
  for(x=[-8,8]) translate([x,0,2]) cylinder(d=2.5,h=12);
  translate([0,-2,8]) rotate([18,0,0]) cube([14,10,8],center=true);
 }
}
module assembly(){
 color("dimgray") translate([0,0,0]) bottom_shell();
 color("gray") translate([0,0,0]) top_shell();
 for(a=[45,135,225,315]){
   rotate([0,0,a]) translate([0,0,body_h/2]) color("slategray") arm_one();
   rotate([0,0,a]) translate([0,68,body_h/2+arm_h/2]) color("black") guard();
   rotate([0,0,a]) translate([0,68,body_h/2+arm_h+1]) color("silver") propeller();
 }
 translate([0,-body_l/2-10,0]) camera_mount();
}
if(PART=="assembly") assembly();
if(PART=="top") top_shell();
if(PART=="bottom") bottom_shell();
if(PART=="arm") arm_one();
if(PART=="rotor_guard") guard();
if(PART=="propeller") propeller();
if(PART=="camera_mount") camera_mount();
