// Modular guarded quadcopter demo — parametric concept, dimensions mm
// Print-bed target: each exported component must be oriented and checked <=200x200 mm.
// PART: assembly, top, bottom, arm, rotor_guard, propeller, camera_mount
$fn=72;
PART="assembly";
body_w=104; body_l=112; body_h=26; wall=2.4;
guard_od=72; guard_id=64; guard_h=18; arm_w=14; arm_h=8;
prop_d=58; prop_t=3.0; hub_d=12; shaft_d=3.2;
screw_d=3.2; boss_d=8; camera_w=24; camera_h=18;
module rr(size,r=5){ hull(){ for(x=[-size[0]/2+r,size[0]/2-r],y=[-size[1]/2+r,size[1]/2-r]) translate([x,y,0]) cylinder(r=r,h=size[2],center=true); } }
module body_holes(z0,z1){for(x=[-body_w/2+8,body_w/2-8],y=[-body_l/2+8,body_l/2-8]) translate([x,y,z0-1]) cylinder(d=screw_d,h=z1-z0+2);}
module bottom_shell(){
 difference(){
  union(){
   translate([0,0,4]) rr([body_w,body_l,8],7);
   // flat floor and continuous shallow electronics tray
   translate([0,0,body_h/2]) difference(){rr([body_w,body_l,body_h],7);translate([0,0,wall]) rr([body_w-2*wall,body_l-2*wall,body_h],5);}
   for(x=[-body_w/2+8,body_w/2-8],y=[-body_l/2+8,body_l/2-8]) translate([x,y,body_h/2]) cylinder(d=boss_d,h=body_h);
  }
  body_holes(-1,body_h+2);
 }
}
module top_shell(){
 difference(){
  union(){
   translate([0,0,body_h+4]) rr([body_w,body_l,8],7);
   translate([0,0,body_h+9]) difference(){rr([body_w,body_l,10],7);translate([0,0,-wall]) rr([body_w-2*wall,body_l-2*wall,10],5);}
   for(x=[-body_w/2+8,body_w/2-8],y=[-body_l/2+8,body_l/2-8]) translate([x,y,body_h+1]) cylinder(d=10,h=8);
  }
  body_holes(body_h,body_h+15);
 }
}
module arm(){
 // One arm inserts between body halves. Motor pattern is deliberately generic.
 difference(){
  union(){
   translate([0,29,arm_h/2]) cube([arm_w,74,arm_h],center=true);
   translate([0,0,arm_h/2]) cube([body_w/2-2,22,arm_h],center=true);
   translate([0,65,arm_h/2]) cylinder(d=guard_od+7,h=arm_h,center=true);
   translate([0,65,arm_h/2]) cylinder(d=24,h=arm_h+4,center=true);
  }
  translate([0,0,-1]) cylinder(d=screw_d,h=arm_h+2);
  for(x=[-6,6]) translate([x,8,-1]) cylinder(d=screw_d,h=arm_h+2);
  translate([0,65,-1]) cylinder(d=16,h=arm_h+2);
  for(x=[-4,4]) translate([x,65,-1]) cylinder(d=2.5,h=arm_h+2);
 }
}
module rotor_guard(){
 // Printable annular guard with radial ribs connecting inner mounting boss to ring
 difference(){
  union(){
   difference(){cylinder(d=guard_od,h=guard_h);translate([0,0,-1]) cylinder(d=guard_id,h=guard_h+2);}
   for(a=[0:90:359]) rotate([0,0,a]) translate([guard_id/4,0,guard_h/2]) cube([guard_id/2,2.4,guard_h],center=true);
   cylinder(d=24,h=guard_h);
  }
  translate([0,0,-1]) cylinder(d=16,h=guard_h+2);
  for(a=[0:90:359]) rotate([0,0,a]) translate([10,0,-1]) cylinder(d=2.5,h=guard_h+2);
 }
}
// Printable 3-blade prototype: thickened swept planform, rounded roots/tips.
// This is a manufacturable prototype shape, NOT a validated flight propeller.
// Use low RPM guarded tests only after inspection/balancing; commercial props are safer.
module one_blade(){
 hull(){
  translate([8,0,0]) cylinder(d=7,h=prop_t);
  translate([14,1,0]) cylinder(d=6.5,h=prop_t);
  translate([prop_d/2-6,5,0]) cylinder(d=4.5,h=prop_t);
  translate([prop_d/2-3,2,0]) cylinder(d=3.8,h=prop_t);
 }
}
module propeller(){
 difference(){
  union(){
   cylinder(d=hub_d,h=prop_t);
   for(a=[0:120:359]) rotate([0,0,a]) one_blade();
  }
  translate([0,0,-1]) cylinder(d=shaft_d,h=prop_t+2);
  // optional two tiny balancing/retention reliefs
  for(a=[0:180:179]) rotate([0,0,a]) translate([5,0,-1]) cylinder(d=1.5,h=prop_t+2);
 }
}
module camera_mount(){
 difference(){
  hull(){translate([0,0,4]) cube([camera_w,camera_h,3],center=true);translate([0,-4,10]) cube([camera_w,8,3],center=true);}
  for(x=[-8,8]) translate([x,0,2]) cylinder(d=2.5,h=12);
  translate([0,-2,8]) rotate([18,0,0]) cube([14,10,8],center=true);
 }
}
module assembly(){
 color("dimgray") bottom_shell();
 color("gray") top_shell();
 for(a=[45,135,225,315]){
  rotate([0,0,a]) translate([0,0,body_h/2]) color("slategray") arm();
  rotate([0,0,a]) translate([0,65,body_h/2+arm_h/2]) color("black") rotor_guard();
  rotate([0,0,a]) translate([0,65,body_h/2+arm_h+1]) color("silver") propeller();
 }
 translate([0,-body_l/2-10,0]) camera_mount();
}
if(PART=="assembly") assembly();
if(PART=="top") top_shell();
if(PART=="bottom") bottom_shell();
if(PART=="arm") arm();
if(PART=="rotor_guard") rotor_guard();
if(PART=="propeller") propeller();
if(PART=="camera_mount") camera_mount();
