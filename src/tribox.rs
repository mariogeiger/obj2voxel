/********************************************************/
/* AABB-triangle overlap test code                      */
/* by Tomas Akenine-Möller                              */
/* translated into Rust by Mario Geiger                 */
/* Function: int triBoxOverlap(float boxcenter[3],      */
/*          float boxhalfsize[3],float triverts[3][3]); */
/* History:                                             */
/*   2001-03-05: released the code in its first version */
/*   2001-06-18: changed the order of the tests, faster */
/*                                                      */
/* Acknowledgement: Many thanks to Pierre Terdiman for  */
/* suggestions and discussions on how to optimize code. */
/* Thanks to David Hunt for finding a ">="-bug!         */
/********************************************************/

const X: usize = 0;
const Y: usize = 1;
const Z: usize = 2;

fn cross(v1: &[f32; 3], v2: &[f32; 3]) -> [f32; 3] {
    [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ]
}

fn dot(v1: &[f32; 3], v2: &[f32; 3]) -> f32 {
    v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
}

fn sub(v1: &[f32; 3], v2: &[f32; 3]) -> [f32; 3] {
    [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]]
}

fn min_max2(x0: f32, x1: f32) -> (f32, f32) {
    if x0 < x1 {
        (x0, x1)
    } else {
        (x1, x0)
    }
}

fn min_max3(x0: f32, x1: f32, x2: f32) -> (f32, f32) {
    let mut min = x0;
    let mut max = x0;
    if x1 < min {
        min = x1;
    }
    if x1 > max {
        max = x1;
    }
    if x2 < min {
        min = x2;
    }
    if x2 > max {
        max = x2;
    }
    (min, max)
}

fn plane_box_overlap(normal: &[f32; 3], vert: &[f32; 3], maxbox: &[f32; 3]) -> bool {
    let mut vmin: [f32; 3] = [0.0; 3];
    let mut vmax: [f32; 3] = [0.0; 3];

    for q in 0..3 {
        if normal[q] > 0.0 {
            vmin[q] = -maxbox[q] - vert[q];
            vmax[q] = maxbox[q] - vert[q];
        } else {
            vmin[q] = maxbox[q] - vert[q];
            vmax[q] = -maxbox[q] - vert[q];
        }
    }

    if dot(normal, &vmin) > 0.0 {
        return false;
    }

    if dot(normal, &vmax) >= 0.0 {
        return true;
    }

    false
}

pub fn tri_box_overlap(
    boxcenter: &[f32; 3],
    boxhalfsize: &[f32; 3],
    triverts: &[[f32; 3]; 3],
) -> bool {
    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
    /*       this gives 3x3=9 more tests */

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */
    let v0 = sub(&triverts[0], boxcenter);
    let v1 = sub(&triverts[1], boxcenter);
    let v2 = sub(&triverts[2], boxcenter);

    /* compute triangle edges */
    let e0 = sub(&v1, &v0); /* tri edge 0 */
    let e1 = sub(&v2, &v1); /* tri edge 1 */
    let e2 = sub(&v0, &v2); /* tri edge 2 */

    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */

    // #define AXISTEST_X01(a, b, fa, fb)			   \
    // 	p0 = a*v0[Y] - b*v0[Z];			       	   \
    // 	p2 = a*v2[Y] - b*v2[Z];			       	   \
    //         if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
    // 	rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
    // 	if(min>rad || max<-rad) return 0;
    let axistest_x01 = |a: f32, b: f32, fa: f32, fb: f32| -> bool {
        let p0 = a * v0[Y] - b * v0[Z];
        let p2 = a * v2[Y] - b * v2[Z];
        let (min, max) = min_max2(p0, p2);
        let rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];
        min > rad || max < -rad
    };
    //
    // #define AXISTEST_X2(a, b, fa, fb)			   \
    // 	p0 = a*v0[Y] - b*v0[Z];			           \
    // 	p1 = a*v1[Y] - b*v1[Z];			       	   \
    //         if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    // 	rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];   \
    // 	if(min>rad || max<-rad) return 0;
    let axistest_x2 = |a: f32, b: f32, fa: f32, fb: f32| -> bool {
        let p0 = a * v0[Y] - b * v0[Z];
        let p1 = a * v1[Y] - b * v1[Z];
        let (min, max) = min_max2(p0, p1);
        let rad = fa * boxhalfsize[Y] + fb * boxhalfsize[Z];
        min > rad || max < -rad
    };
    //
    // #define AXISTEST_Y02(a, b, fa, fb)			   \
    // 	p0 = -a*v0[X] + b*v0[Z];		      	   \
    // 	p2 = -a*v2[X] + b*v2[Z];	       	       	   \
    //         if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
    // 	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
    // 	if(min>rad || max<-rad) return 0;
    let axistest_y02 = |a: f32, b: f32, fa: f32, fb: f32| -> bool {
        let p0 = -a * v0[X] + b * v0[Z];
        let p2 = -a * v2[X] + b * v2[Z];
        let (min, max) = min_max2(p0, p2);
        let rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];
        min > rad || max < -rad
    };
    //
    // #define AXISTEST_Y1(a, b, fa, fb)			   \
    // 	p0 = -a*v0[X] + b*v0[Z];		      	   \
    // 	p1 = -a*v1[X] + b*v1[Z];	     	       	   \
    //         if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    // 	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];   \
    // 	if(min>rad || max<-rad) return 0;
    let axistest_y1 = |a: f32, b: f32, fa: f32, fb: f32| -> bool {
        let p0 = -a * v0[X] + b * v0[Z];
        let p1 = -a * v1[X] + b * v1[Z];
        let (min, max) = min_max2(p0, p1);
        let rad = fa * boxhalfsize[X] + fb * boxhalfsize[Z];
        min > rad || max < -rad
    };
    //
    // #define AXISTEST_Z12(a, b, fa, fb)			   \
    // 	p1 = a*v1[X] - b*v1[Y];			           \
    // 	p2 = a*v2[X] - b*v2[Y];			       	   \
    //         if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} \
    // 	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
    // 	if(min>rad || max<-rad) return 0;
    let axistest_z12 = |a: f32, b: f32, fa: f32, fb: f32| -> bool {
        let p1 = a * v1[X] - b * v1[Y];
        let p2 = a * v2[X] - b * v2[Y];
        let (min, max) = min_max2(p1, p2);
        let rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];
        min > rad || max < -rad
    };
    //
    // #define AXISTEST_Z0(a, b, fa, fb)			   \
    // 	p0 = a*v0[X] - b*v0[Y];				   \
    // 	p1 = a*v1[X] - b*v1[Y];			           \
    //         if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    // 	rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];   \
    // 	if(min>rad || max<-rad) return 0;
    let axistest_z0 = |a: f32, b: f32, fa: f32, fb: f32| -> bool {
        let p0 = a * v0[X] - b * v0[Y];
        let p1 = a * v1[X] - b * v1[Y];
        let (min, max) = min_max2(p0, p1);
        let rad = fa * boxhalfsize[X] + fb * boxhalfsize[Y];
        min > rad || max < -rad
    };

    let fex = f32::abs(e0[X]);
    let fey = f32::abs(e0[Y]);
    let fez = f32::abs(e0[Z]);
    // AXISTEST_X01(e0[Z], e0[Y], fez, fey);
    // AXISTEST_Y02(e0[Z], e0[X], fez, fex);
    // AXISTEST_Z12(e0[Y], e0[X], fey, fex);
    if axistest_x01(e0[Z], e0[Y], fez, fey) {
        return false;
    }
    if axistest_y02(e0[Z], e0[X], fez, fex) {
        return false;
    }
    if axistest_z12(e0[Y], e0[X], fey, fex) {
        return false;
    }

    let fex = f32::abs(e1[X]);
    let fey = f32::abs(e1[Y]);
    let fez = f32::abs(e1[Z]);
    // AXISTEST_X01(e1[Z], e1[Y], fez, fey);
    // AXISTEST_Y02(e1[Z], e1[X], fez, fex);
    // AXISTEST_Z0(e1[Y], e1[X], fey, fex);
    if axistest_x01(e1[Z], e1[Y], fez, fey) {
        return false;
    }
    if axistest_y02(e1[Z], e1[X], fez, fex) {
        return false;
    }
    if axistest_z0(e1[Y], e1[X], fey, fex) {
        return false;
    }

    let fex = f32::abs(e2[X]);
    let fey = f32::abs(e2[Y]);
    let fez = f32::abs(e2[Z]);
    // AXISTEST_X2(e2[Z], e2[Y], fez, fey);
    // AXISTEST_Y1(e2[Z], e2[X], fez, fex);
    // AXISTEST_Z12(e2[Y], e2[X], fey, fex);
    if axistest_x2(e2[Z], e2[Y], fez, fey) {
        return false;
    }
    if axistest_y1(e2[Z], e2[X], fez, fex) {
        return false;
    }
    if axistest_z12(e2[Y], e2[X], fey, fex) {
        return false;
    }

    /* Bullet 1: */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in X-direction */
    let (min, max) = min_max3(v0[X], v1[X], v2[X]);
    if min > boxhalfsize[X] || max < -boxhalfsize[X] {
        return false;
    }

    /* test in Y-direction */
    let (min, max) = min_max3(v0[Y], v1[Y], v2[Y]);
    if min > boxhalfsize[Y] || max < -boxhalfsize[Y] {
        return false;
    }

    /* test in Z-direction */
    let (min, max) = min_max3(v0[Z], v1[Z], v2[Z]);
    if min > boxhalfsize[Z] || max < -boxhalfsize[Z] {
        return false;
    }

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    if !plane_box_overlap(&cross(&e0, &e1), &v0, boxhalfsize) {
        return false;
    }

    true
}
