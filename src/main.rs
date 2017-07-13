#[macro_use]
extern crate npy_derive;
extern crate npy;
extern crate obj;
extern crate clap;
extern crate nalgebra as na;
extern crate kiss3d;
use std::path::Path;

use kiss3d::window::Window;
use kiss3d::light::Light;
use na::Translation3;

mod tribox;

fn voxel_grid(size: usize, border: usize, vertices: &[[f32; 3]]) -> ([f32; 3], f32) {
    let mut bounding_min = vertices[0];
    let mut bounding_max = vertices[0];
    for point in vertices.iter() {
        for i in 0..3 {
            if bounding_min[i] > point[i] {
                bounding_min[i] = point[i];
            }
            if bounding_max[i] < point[i] {
                bounding_max[i] = point[i];
            }
        }
    }
    let cube_size: f32 = (0..3).map(|i| bounding_max[i] - bounding_min[i]).fold(0.0, f32::max) /
                         ((size - 2 * border) as f32);

    let mut origin = [0.0; 3];
    for dim in 0..3 {
        let center = 0.5 * (bounding_min[dim] + bounding_max[dim]);
        origin[dim] = center - cube_size * size as f32 / 2.0;
    }
    (origin, cube_size)
}

fn tri_voxel_overlap(triverts: &[[f32; 3]; 3],
                     origin: &[f32; 3],
                     cube_size: f32,
                     size: usize)
                     -> Vec<usize> {
    let mut tri_min = triverts[0];
    let mut tri_max = triverts[0];
    for i in 0..3 {
        if tri_max[i] < triverts[1][i] {
            tri_max[i] = triverts[1][i];
        }
        if tri_min[i] > triverts[1][i] {
            tri_min[i] = triverts[1][i];
        }
        if tri_max[i] < triverts[2][i] {
            tri_max[i] = triverts[2][i];
        }
        if tri_min[i] > triverts[2][i] {
            tri_min[i] = triverts[2][i];
        }
    }

    let ijk_min: Vec<usize> = (0..3)
        .map(|dim| {
            let begin = f32::floor((tri_min[dim] - origin[dim]) / cube_size - 0.1) as isize;
            if begin < 0 { 0usize } else { begin as usize }
        })
        .collect();
    let ijk_max: Vec<usize> = (0..3)
        .map(|dim| {
            let end = f32::ceil((tri_max[dim] - origin[dim]) / cube_size + 0.1) as usize;
            if end > size { size } else { end }
        })
        .collect();

    let mut output = Vec::new();
    for i in ijk_min[0]..ijk_max[0] {
        for j in ijk_min[1]..ijk_max[1] {
            for k in ijk_min[2]..ijk_max[2] {
                let boxcenter = [origin[0] + i as f32 * cube_size + cube_size / 2.0,
                                 origin[1] + j as f32 * cube_size + cube_size / 2.0,
                                 origin[2] + k as f32 * cube_size + cube_size / 2.0];
                if tribox::tri_box_overlap(&boxcenter,
                                           &[cube_size / 2.0, cube_size / 2.0, cube_size / 2.0],
                                           triverts) {
                    output.push(i * size * size + j * size + k);
                }
            }
        }
    }
    output
}

fn save_to(voxel: &[u8], file: &str) {
    #[derive(NpyData)]
    struct VoxelValue {
        field: u8,
    }
    let data: Vec<VoxelValue> = voxel.iter().map(|&x| VoxelValue { field: x }).collect();
    npy::to_file(file, data.iter()).unwrap();
}

fn visualize(voxel: &[u8], size: usize) {
    let mut window = Window::new("view");

    let cube_size = 0.5 / size as f32;
    let origin = -0.5 * cube_size * size as f32;

    for x in 0..size {
        for y in 0..size {
            for z in 0..size {
                if voxel[x * size * size + y * size + z] != 0 {
                    let cx = origin + cube_size * x as f32;
                    let cy = origin + cube_size * y as f32;
                    let cz = origin + cube_size * z as f32;

                    let mut c = window.add_cube(cube_size, cube_size, cube_size);
                    c.append_translation(&Translation3::new(cx, cy, cz));
                    if ((x % 2) + (y % 2) + (z % 2)) % 2 == 0 {
                        c.set_color(0.7, 0.7, 1.0);
                    } else {
                        c.set_color(0.7, 1.0, 0.7);
                    }
                }
            }
        }
    }
    window.set_light(Light::StickToCamera);
    while window.render() {}
}

fn main() {
    let matches = clap::App::new("obj2voxel")
        .version("1.0")
        .author("Mario <geiger.mario@gmail.com>")
        .about("Convert Obj file into voxel")
        .arg(clap::Arg::with_name("size")
            .short("s")
            .long("size")
            .value_name("SIZE")
            .required(true)
            .help("Output a SIZExSIZExSIZE voxel")
            .takes_value(true))
        .arg(clap::Arg::with_name("border")
            .short("b")
            .long("border")
            .value_name("B")
            .required(false)
            .help("Add a border of empty voxels around the result. The output size remain SIZE but the object occupies only SIZE - 2 B.")
            .takes_value(true))
        .arg(clap::Arg::with_name("INPUT")
            .help("The input Obj file to read")
            .required(true)
            .index(1))
        .arg(clap::Arg::with_name("OUTPUT")
            .help("The output numpy file to write in")
            .required(false)
            .index(2))
        .arg(clap::Arg::with_name("view")
            .short("v")
            .long("view")
            .help("Visualize the result in 3D"))
        .get_matches();

    let size: usize = matches.value_of("size").unwrap().parse().unwrap();
    let mut voxel = vec![0u8; size * size * size];

    let o = obj::load::<obj::SimplePolygon>(Path::new(matches.value_of("INPUT").unwrap())).unwrap();

    let border: usize = matches.value_of("border").unwrap_or("0").parse().unwrap();
    let (origin, cube_size) = voxel_grid(size, border, &o.position);

    for object in o.objects {
        for group in object.groups {
            for face in group.indices {
                assert!(face.len() == 3, "not made of triangles");

                let triverts = [o.position[face[0].0], o.position[face[1].0], o.position[face[2].0]];

                for i in tri_voxel_overlap(&triverts, &origin, cube_size, size) {
                    voxel[i] = 1;
                }
            }
        }
    }

    if matches.is_present("OUTPUT") {
        save_to(&voxel, matches.value_of("OUTPUT").unwrap());
    }

    if matches.is_present("view") {
        visualize(&voxel, size);
    }
}
