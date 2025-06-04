# Mesh Kernel Computation

A C++ tool to compute the **kernel of a 3D mesh**, using robust geometric algorithms and optional visualization features. This project supports various options for customizing the computation pipeline and output format.

TODO:
- Implementation of "Exact and Efficient Mesh-Kernel Generation" - J. Nehring-Wirxel and P. Kern and P. Trettner and L. Kobbelt
- how to cite us

## Features

* Compute the **kernel polyhedron** of a 3D mesh.
* Optional use of **exact linear programming** or **Seidel's solver** for feasibility checks.
* Rendering of input and result meshes (optional).

## Build Instructions

This is a C++ project that uses **CMake** as its build system.

```bash
git clone --recursive https://github.com/juliusnehring/mesh-kernel.git
cd mesh-kernel
mkdir build
cd build
cmake ..
make
```

## Usage

```bash
./mesh_kernel -i input_mesh.obj -o result_mesh.obj
```

### Options

| Flag                        | Description                                                                             |
| --------------------------- | --------------------------------------------------------------------------------------- |
| `-i, --input`               | Path to input mesh (required)                                                           |
| `-o, --output`              | Path to output mesh (required)                                                          |
| `-e, --extension`           | Output file extension: `obj` or `stl` (default: `obj`)                                  |
| `--show-input`              | Render the input mesh                                                                   |
| `--show-result`             | Render the result (kernel) mesh                                                         |
| `--disable-exact-lp`        | Disable that the exact LP is run in parallel, only plane cutting happens in this mode   |
| `--check-exact-feasibility` | Only check if the kernel exists using Seidel's solver, no kernel polyhedron computation |
| `--use-uset`                | Use `unordered_set` to compute unique cutting planes                                    |
| `--disable-kdop`            | Disable kdop-based culling                                                              |
| `-k, --kdop-k`              | Set kdop `k` parameter (default: `3`, which corresponds to AABB)                        |
| `--triangulate`             | Triangulate the output mesh                                                             |

### Example

```bash
./mesh_kernel -i bunny.obj -o bunny_kernel.obj --triangulate
```

This command computes the kernel of the `bunny.obj` mesh, using Seidel's solver for early-out checks and saves the triangulated result as `bunny_kernel.obj`.

<!-- ## License -->

<!-- [MIT](LICENSE) -->

<!-- --- -->
