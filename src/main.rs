extern crate clap;
#[cfg(feature = "viewer")]
extern crate kiss3d;
extern crate nalgebra as na;
extern crate npy;
extern crate obj;
extern crate rand;
extern crate rand_distr;
use obj::Obj;
use rand_distr::{Distribution, Normal, Uniform};
use std::path::Path;

#[cfg(feature = "viewer")]
use kiss3d::light::Light;
#[cfg(feature = "viewer")]
use kiss3d::nalgebra::Translation3;
#[cfg(feature = "viewer")]
use kiss3d::window::Window;

mod tribox;

fn bounding_box(vertices: &[[f32; 3]]) -> ([f32; 3], [f32; 3]) {
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
    (bounding_min, bounding_max)
}

fn voxel_grid(size: usize, border: usize, vertices: &[[f32; 3]]) -> ([f32; 3], f32) {
    let (bounding_min, bounding_max) = bounding_box(vertices);

    let cube_size: f32 = (0..3)
        .map(|i| bounding_max[i] - bounding_min[i])
        .fold(0.0, f32::max)
        / ((size - 2 * border) as f32);

    let mut origin = [0.0; 3];
    for dim in 0..3 {
        let center = 0.5 * (bounding_min[dim] + bounding_max[dim]);
        origin[dim] = center - cube_size * size as f32 / 2.0;
    }
    (origin, cube_size)
}

fn voxel_grid_cube_size(size: usize, vertices: &[[f32; 3]], cube_size: f32) -> ([f32; 3], f32) {
    let (bounding_min, bounding_max) = bounding_box(vertices);

    let mut origin = [0.0; 3];
    for dim in 0..3 {
        let center = 0.5 * (bounding_min[dim] + bounding_max[dim]);
        origin[dim] = center - cube_size * size as f32 / 2.0;
    }
    (origin, cube_size)
}

fn diagonal_bb_cube_size(size: usize, border: usize, vertices: &[[f32; 3]]) -> f32 {
    let (bounding_min, bounding_max) = bounding_box(vertices);

    let cube_size: f32 = (0..3)
        .map(|i| bounding_max[i] - bounding_min[i])
        .map(|x| x.powi(2))
        .sum::<f32>()
        .sqrt()
        / ((size - 2 * border) as f32);

    cube_size
}

fn diagonal_bb_xy_cube_size(size: usize, border: usize, vertices: &[[f32; 3]]) -> f32 {
    let (bounding_min, bounding_max) = bounding_box(vertices);

    let cube_size: f32 = f32::max(
        (0..2)
            .map(|i| bounding_max[i] - bounding_min[i])
            .map(|x| x.powi(2))
            .sum::<f32>()
            .sqrt(),
        bounding_max[2] - bounding_min[2],
    ) / ((size - 2 * border) as f32);

    cube_size
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
    let matches = clap::Command::new("obj2voxel")
        .version("1.0")
        .author("Mario <geiger.mario@gmail.com>")
        .about("Convert Obj file into voxel")
        .arg(
            clap::Arg::new("INPUT")
                .help("The input Obj file to read")
                .required(true)
                .index(1),
        )
        .arg(
            clap::Arg::new("OUTPUT")
                .help("The output numpy file to write in")
                .required(false)
                .index(2),
        )
        .arg(
            clap::Arg::new("size")
                .long("size")
                .value_name("SIZE")
                .required(true)
                .help("Output a SIZExSIZExSIZE voxel")
                .takes_value(true),
        )
        .arg(
            clap::Arg::new("border")
                .long("border")
                .value_name("B")
                .help(
                    "Add a border of empty voxels around the result. The output size remain SIZE \
                     but the object occupies only SIZE - 2 B.",
                )
                .takes_value(true),
        )
        .arg(
            clap::Arg::new("view")
                .short('v')
                .long("view")
                .help("Visualize the result in 3D"),
        )
        .arg(
            clap::Arg::new("double")
                .long("double")
                .help("Each cube is redered from 8 smaller cubes"),
        )
        .arg(
            clap::Arg::new("rotate")
                .long("rotate")
                .help("Apply a random rotation"),
        )
        .arg(
            clap::Arg::new("alpha_rot")
                .long("alpha_rot")
                .value_name("ALPHA")
                .default_value("0.0")
                .help("Apply a rotation around Z axis")
                .takes_value(true),
        )
        .arg(
            clap::Arg::new("beta_rot")
                .long("beta_rot")
                .value_name("BETA")
                .default_value("0.0")
                .help("Apply a rotation around Y axis")
                .takes_value(true),
        )
        .arg(
            clap::Arg::new("gamma_rot")
                .long("gamma_rot")
                .value_name("GAMMA")
                .default_value("0.0")
                .help("Apply a rotation around Z axis")
                .takes_value(true),
        )
        .arg(
            clap::Arg::new("diagonal_bounding_box_xy")
                .long("diagonal_bounding_box_xy")
                .help(
                    "Compute the cube size from the diagonal of the original BB in xy directions",
                ),
        )
        .arg(
            clap::Arg::new("diagonal_bounding_box")
                .long("diagonal_bounding_box")
                .help("Compute the cube size from the diagonal of the original BB"),
        )
        .get_matches();

    let size: usize = matches.value_of("size").unwrap().parse().unwrap();
    let mut voxel = vec![0u8; size * size * size];

    let file = Path::new(matches.value_of("INPUT").unwrap());
    let mut obj = Obj::load(file).expect(&format!("Cannot open {:?}", file));

    let border: usize = matches.value_of("border").unwrap_or("0").parse().unwrap();
    let double = matches.is_present("double");
    let cube_size = if matches.is_present("diagonal_bounding_box_xy") {
        diagonal_bb_xy_cube_size(
            if double { 2 * size } else { size },
            border,
            &obj.data.position,
        )
    } else {
        diagonal_bb_cube_size(
            if double { 2 * size } else { size },
            border,
            &obj.data.position,
        )
    };

    fn rotate_obj(obj: &mut Obj, rot: &na::Rotation3<f32>) {
        for position in &mut obj.data.position {
            let vec = rot * na::Vector3::new(position[0], position[1], position[2]);
            position[0] = vec[0];
            position[1] = vec[1];
            position[2] = vec[2];
        }
        for normal in &mut obj.data.normal {
            let vec = rot * na::Vector3::new(normal[0], normal[1], normal[2]);
            normal[0] = vec[0];
            normal[1] = vec[1];
            normal[2] = vec[2];
        }
    }

    rotate_obj(
        &mut obj,
        &na::Rotation3::new(na::Vector3::new(
            0.0,
            0.0,
            matches
                .value_of("gamma_rot")
                .unwrap_or("0")
                .parse()
                .unwrap(),
        )),
    );
    rotate_obj(
        &mut obj,
        &na::Rotation3::new(na::Vector3::new(
            0.0,
            matches.value_of("beta_rot").unwrap_or("0").parse().unwrap(),
            0.0,
        )),
    );
    rotate_obj(
        &mut obj,
        &na::Rotation3::new(na::Vector3::new(
            0.0,
            0.0,
            matches
                .value_of("alpha_rot")
                .unwrap_or("0")
                .parse()
                .unwrap(),
        )),
    );

    if matches.is_present("rotate") {
        let mut rng = rand::thread_rng();
        let die = Normal::new(0.0, 1.0f32).unwrap();
        let a = na::Vector3::new(
            die.sample(&mut rng),
            die.sample(&mut rng),
            die.sample(&mut rng),
        );
        let die = Uniform::new(0.0f32, std::f32::consts::PI);
        let a = die.sample(&mut rng) * a / a.norm();
        let r = na::Rotation3::new(a);
        rotate_obj(&mut obj, &r);
    }

    let (origin, cube_size) = if matches.is_present("diagonal_bounding_box")
        || matches.is_present("diagonal_bounding_box_xy")
    {
        voxel_grid_cube_size(
            if double { 2 * size } else { size },
            &obj.data.position,
            cube_size,
        )
    } else {
        voxel_grid(
            if double { 2 * size } else { size },
            border,
            &obj.data.position,
        )
    };

    for object in obj.data.objects {
        for group in object.groups {
            for face in group.polys {
                for k in 1..face.0.len() - 1 {
                    let triverts = [
                        obj.data.position[face.0[0].0],
                        obj.data.position[face.0[k].0],
                        obj.data.position[face.0[k + 1].0],
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
