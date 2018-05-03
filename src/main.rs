#![cfg_attr(feature = "clippy", feature(plugin))]
#![cfg_attr(feature = "clippy", plugin(clippy))]

extern crate clap;
#[cfg(feature = "viewer")]
extern crate kiss3d;
extern crate nalgebra as na;
extern crate npy;
extern crate obj;
extern crate rand;
use obj::{Obj, SimplePolygon};
use std::path::Path;

#[cfg(feature = "viewer")]
use kiss3d::light::Light;
#[cfg(feature = "viewer")]
use kiss3d::window::Window;
#[cfg(feature = "viewer")]
use na::Translation3;
use rand::Rand;

mod tribox;

fn voxel_grid(size: usize, border: usize, vertices: &[[f32; 3]]) -> ([f32; 3], f32) {
    let mut bounding_min = vertices[0];
    let mut bounding_max = vertices[0];
    for vert in vertices.iter() {
        for dim in 0..3 {
            if bounding_min[dim] > vert[dim] {
                bounding_min[dim] = vert[dim];
            }
            if bounding_max[dim] < vert[dim] {
                bounding_max[dim] = vert[dim];
            }
        }
    }
    let cube_size: f32 = (0..3)
        .map(|i| bounding_max[i] - bounding_min[i])
        .fold(0.0, f32::max) / ((size - 2 * border) as f32);

    let mut origin = [0.0; 3];
    for dim in 0..3 {
        let center = 0.5 * (bounding_min[dim] + bounding_max[dim]);
        origin[dim] = center - cube_size * size as f32 / 2.0;
    }
    (origin, cube_size)
}

fn tri_voxel_overlap(
    triverts: &[[f32; 3]; 3],
    origin: &[f32; 3],
    cube_size: f32,
    size: usize,
) -> Vec<(usize, usize, usize)> {
    let mut tri_min = triverts[0];
    let mut tri_max = triverts[0];
    for vert in triverts {
        for dim in 0..3 {
            if tri_min[dim] > vert[dim] {
                tri_min[dim] = vert[dim];
            }
            if tri_max[dim] < vert[dim] {
                tri_max[dim] = vert[dim];
            }
        }
    }

    let ijk_min: Vec<usize> = (0..3)
        .map(|dim| {
            let begin = f32::floor((tri_min[dim] - origin[dim]) / cube_size - 0.1) as isize;
            if begin < 0 {
                0usize
            } else {
                begin as usize
            }
        })
        .collect();
    let ijk_max: Vec<usize> = (0..3)
        .map(|dim| {
            let end = f32::ceil((tri_max[dim] - origin[dim]) / cube_size + 0.1) as usize;
            if end > size {
                size
            } else {
                end
            }
        })
        .collect();

    let mut output = Vec::new();
    for i in ijk_min[0]..ijk_max[0] {
        for j in ijk_min[1]..ijk_max[1] {
            for k in ijk_min[2]..ijk_max[2] {
                let boxcenter = [
                    origin[0] + i as f32 * cube_size + cube_size / 2.0,
                    origin[1] + j as f32 * cube_size + cube_size / 2.0,
                    origin[2] + k as f32 * cube_size + cube_size / 2.0,
                ];
                if tribox::tri_box_overlap(
                    &boxcenter,
                    &[cube_size / 2.0, cube_size / 2.0, cube_size / 2.0],
                    triverts,
                ) {
                    output.push((i, j, k));
                }
            }
        }
    }
    output
}

fn save_to(voxel: &[u8], file: &str) {
    npy::to_file(file, voxel.iter().cloned()).unwrap();
}

#[cfg(feature = "viewer")]
fn visualize(voxel: &[u8], size: usize) {
    let mut window = Window::new("view");

    let cube_size = 0.5 / size as f32;
    let origin = -0.5 * cube_size * size as f32;

    for x in 0..size {
        for y in 0..size {
            for z in 0..size {
                let value = voxel[x * size * size + y * size + z];
                if value != 0 {
                    let cx = origin + cube_size * x as f32;
                    let cy = origin + cube_size * y as f32;
                    let cz = origin + cube_size * z as f32;

                    let mut c = window.add_cube(cube_size, cube_size, cube_size);
                    c.append_translation(&Translation3::new(cx, cy, cz));

                    let color = 1.0 - f32::powi(0.3, value as i32);
                    if ((x % 2) + (y % 2) + (z % 2)) % 2 == 0 {
                        c.set_color(color, color, 1.0);
                    } else {
                        c.set_color(color, 1.0, color);
                    }
                }
            }
        }
    }
    window.set_light(Light::StickToCamera);
    while window.render() {}
}

#[cfg(not(feature = "viewer"))]
fn visualize(_voxel: &[u8], _size: usize) {
    panic!("Not compiled with viewer feature");
}

fn main() {
    let matches = clap::App::new("obj2voxel")
        .version("1.0")
        .author("Mario <geiger.mario@gmail.com>")
        .about("Convert Obj file into voxel")
        .arg(
            clap::Arg::with_name("size")
                .short("s")
                .long("size")
                .value_name("SIZE")
                .required(true)
                .help("Output a SIZExSIZExSIZE voxel")
                .takes_value(true),
        )
        .arg(
            clap::Arg::with_name("border")
                .short("b")
                .long("border")
                .value_name("B")
                .required(false)
                .help(
                    "Add a border of empty voxels around the result. The output size remain SIZE \
                     but the object occupies only SIZE - 2 B.",
                )
                .takes_value(true),
        )
        .arg(
            clap::Arg::with_name("INPUT")
                .help("The input Obj file to read")
                .required(true)
                .index(1),
        )
        .arg(
            clap::Arg::with_name("OUTPUT")
                .help("The output numpy file to write in")
                .required(false)
                .index(2),
        )
        .arg(
            clap::Arg::with_name("view")
                .short("v")
                .long("view")
                .help("Visualize the result in 3D"),
        )
        .arg(
            clap::Arg::with_name("double")
                .short("d")
                .long("double")
                .help("Each cube is redered from 8 smaller cubes"),
        )
        .arg(
            clap::Arg::with_name("rotate")
                .short("r")
                .long("rotate")
                .help("Apply a random rotation"),
        )
        .get_matches();

    let size: usize = matches.value_of("size").unwrap().parse().unwrap();
    let mut voxel = vec![0u8; size * size * size];

    let file = Path::new(matches.value_of("INPUT").unwrap());
    let mut o = Obj::<SimplePolygon>::load(file).expect(&format!("Cannot open {:?}", file));

    if matches.is_present("rotate") {
        let r = na::Rotation3::rand(&mut rand::thread_rng());

        for position in &mut o.position {
            let vec = r * na::Vector3::new(position[0], position[1], position[2]);
            position[0] = vec[0];
            position[1] = vec[1];
            position[2] = vec[2];
        }
        for normal in &mut o.normal {
            let vec = r * na::Vector3::new(normal[0], normal[1], normal[2]);
            normal[0] = vec[0];
            normal[1] = vec[1];
            normal[2] = vec[2];
        }
    }

    let border: usize = matches.value_of("border").unwrap_or("0").parse().unwrap();
    let double = matches.is_present("double");
    let (origin, cube_size) = voxel_grid(if double { 2 * size } else { size }, border, &o.position);

    for object in o.objects {
        for group in object.groups {
            for face in group.polys {
                for k in 1..face.len() - 1 {
                    let triverts = [
                        o.position[face[0].0],
                        o.position[face[k].0],
                        o.position[face[k + 1].0],
                    ];

                    for (i, j, k) in tri_voxel_overlap(
                        &triverts,
                        &origin,
                        cube_size,
                        if double { 2 * size } else { size },
                    ) {
                        if double {
                            // one bit per minicube
                            voxel[(i / 2) * size * size + (j / 2) * size + (k / 2)] |=
                                1 << (i % 2) * 4 + (j % 2) * 2 + (k % 2);
                        } else {
                            voxel[i * size * size + j * size + k] = 1;
                        }
                    }
                }
            }
        }
    }

    if double {
        for value in &mut voxel {
            // count the bits
            *value = ((*value as u64 * 0x200040008001 & 0x111111111111111) % 0xf) as u8;
        }
    }

    if matches.is_present("OUTPUT") {
        save_to(&voxel, matches.value_of("OUTPUT").unwrap());
    }

    if matches.is_present("view") {
        visualize(&voxel, size);
    }
}
