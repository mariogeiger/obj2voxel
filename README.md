# obj2voxel
Convert [.obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file) to voxels

## Install
1. [install rust](https://www.rustup.rs/)
2. `cargo install --git https://github.com/mariogeiger/obj2voxel`

## Usage

generate a 64x64x64 voxel grid and save it into numpy format

```bash
obj2voxel --size 64 diamond.obj output.npy
```

generate voxel grid and visualize it
requires to compile with cargo install --features "viewer"

```bash
obj2voxel --size 16 diamond.obj --view
```

## Build from source

```bash
cargo build --features viewer
cargo run --features viewer -- --size 16 diamond.obj --view
```