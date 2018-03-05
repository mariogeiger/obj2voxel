# obj2voxel
Convert [.obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file) to voxels

## Install
1. [install rust](https://www.rustup.rs/)
2. `cargo install`

## Usage

    # generate a 64x64x64 voxel grid and save it into numpy format
    obj2voxel --size 64 input.obj output.npy
    
    # generate voxel grid and visualize it
    # requires to compile with cargo install --features "viewer"
    obj2voxel --size 64 --view input.obj
