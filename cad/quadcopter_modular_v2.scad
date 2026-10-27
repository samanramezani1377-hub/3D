// Senoobar 3D — modular quadcopter redesign draft
// Units: mm. Built as separate printable parts; reference envelope based on
// quadcopter_tidy.obj bounds (about 144.6 x 150 x 26.2 mm).
// This is a new parametric draft guided by the reference model, not a boolean
// edit of the original mesh. Validate fit, strength, CG and flight safety before use.

part = "assembly"; // [assembly, upper_shell, lower_tray, arm, guard, camera_mount]
$fn = 72;

body_x = 108;
body_y = 88;
body_z = 18;
wall = 2.4;
tray_floor = 2.4;
arm_len = 42;
arm_w = 15;
arm_h = 8;
guard_outer_d = 56;
guard_wall = 3;
guard_h = 8;
motor_hole_d = 16;
screw_d = 3.2;       // M3 clearance
screw_head_d = 6.4;
screw_head_h = 2.4;
camera_tilt = 12;    // degrees downward
camera_width = 24;
camera_depth = 18;
camera_height = 16;

module rounded_box(x,y,z,r=5) {
    hull() for (sx=[-1,1], sy=[-1,1])
        translate([sx*(x/2-r), sy*(y/2-r),0]) cylinder(r=r,h=z);
}
module screw_holes(z0,z1) {
    for (x=[-body_x/2+10, body_x/2-10], y=[-body_y/2+10, body_y/2-10]) {
        translate([x,y,z0-0.2]) cylinder(d=screw_d,h=z1-z0+0.4);
    }
}
module lower_tray() {
    difference() {
        union() {
            rounded_box(body_x,body_y,tray_floor,6);
            // perimeter wall and four reinforced arm sockets
            difference() {
                translate([0,0,tray_floor]) rounded_box(body_x,body_y,7,6);
                translate([0,0,tray_floor-0.1])
                    rounded_box(body_x-2*wall,body_y-2*wall,7.2,4);
            }
            for (sx=[-1,1], sy=[-1,1])
                translate([sx*(body_x/2-12),sy*(body_y/2-12),tray_floor])
                    cube([18,18,8],center=true);
        }
        // Four lower-access screw bores; countersink is on underside.
        for (x=[-body_x/2+10,body_x/2-10], y=[-body_y/2+10,body_y/2-10]) {
            translate([x,y,-0.1]) cylinder(d=screw_head_d,h=screw_head_h+0.2);
            translate([x,y,screw_head_h]) cylinder(d=screw_d,h=20);
        }
        // electronics mounting slots
        for (x=[-20,0,20], y=[-12,12])
            translate([x,y,-0.1]) cylinder(d=3.2,h=tray_floor+0.2);
    }
}
module upper_shell() {
    difference() {
        translate([0,0,body_z-7]) rounded_box(body_x,body_y,7,6);
        translate([0,0,body_z-7.1]) rounded_box(body_x-2*wall,body_y-2*wall,7.2,4);
        screw_holes(body_z-8,body_z+1);
    }
}
module arm_part() {
    // Arm origin at body-side joint; outer end points along +X.
    difference() {
        union() {
            hull() {
                translate([0,0,arm_h/2]) cube([18,arm_w+8,arm_h],center=true);
                translate([arm_len,0,arm_h/2]) cylinder(d=arm_w,h=arm_h,center=true);
            }
            translate([arm_len,0,arm_h/2]) cylinder(d=23,h=arm_h+2,center=true);
            // Guard connection ears
            for (y=[-11,11]) translate([arm_len-7,y,arm_h/2])
                cylinder(d=7,h=arm_h+1,center=true);
        }
        translate([arm_len,0,-0.1]) cylinder(d=motor_hole_d,h=arm_h+3);
        for (y=[-11,11]) translate([arm_len-7,y,-0.1]) cylinder(d=screw_d,h=arm_h+3);
        for (y=[-8,8]) translate([5,y,-0.1]) cylinder(d=screw_d,h=arm_h+3);
    }
}
module guard_part() {
    difference() {
        union() {
            cylinder(d=guard_outer_d,h=guard_h);
            // Four attachment tabs
            for (a=[0:90:270])
                rotate([0,0,a]) translate([guard_outer_d/2-2,0,guard_h/2])
                    cube([12,12,guard_h],center=true);
        }
        translate([0,0,-0.1]) cylinder(d=guard_outer_d-2*guard_wall,h=guard_h+0.2);
        for (a=[0:90:270])
            rotate([0,0,a]) translate([guard_outer_d/2-7,0,-0.1])
                cylinder(d=screw_d,h=guard_h+0.2);
    }
}
module camera_mount() {
    difference() {
        union() {
            // base flange and angled camera cradle
            translate([0,0,2]) rounded_box(34,24,4,3);
            translate([0,0,8]) rotate([0,camera_tilt,0])
                difference() {
                    translate([0,0,camera_height/2])
                        cube([camera_width+6,camera_depth+6,camera_height],center=true);
                    translate([0,0,camera_height/2])
                        cube([camera_width,camera_depth+1,camera_height+1],center=true);
                }
        }
        for (x=[-12,12]) translate([x,0,-0.1]) cylinder(d=screw_d,h=5);
    }
}
module arm_assembly(angle) {
    rotate([0,0,angle]) {
        translate([body_x/2-7,0,4]) arm_part();
        translate([body_x/2-7+arm_len,0,4]) guard_part();
    }
}
if (part=="lower_tray") lower_tray();
else if (part=="upper_shell") upper_shell();
else if (part=="arm") arm_part();
else if (part=="guard") guard_part();
else if (part=="camera_mount") camera_mount();
else {
    color("dimgray") lower_tray();
    color("silver") upper_shell();
    for (a=[45,135,225,315]) arm_assembly(a);
    translate([0,0,body_z]) camera_mount();
}
