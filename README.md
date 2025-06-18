# Mesh Kernel Computation

A C++ tool to compute the **kernel of a 3D mesh** and perform **exact CSG boolean operations** using robust geometric algorithms and optional visualization features. This project supports various options for customizing the computation pipeline and output format.

## Key Features

* **Mesh Kernel Computation**: Compute the kernel polyhedron of a 3D mesh
* **EMBER CSG Operations**: Exact mesh boolean operations (union, intersection, difference)
* **Exact Arithmetic**: Robust geometric computations using integer-plane-geometry
* **Multiple Input Formats**: Support for OBJ, STL, and other common mesh formats

TODO:
- Implementation of "Exact and Efficient Mesh-Kernel Generation" - J. Nehring-Wirxel and P. Kern and P. Trettner and L. Kobbelt
- how to cite us

## Features

* Compute the **kernel polyhedron** of a 3D mesh.
* Perform **exact CSG boolean operations** (union, intersection, difference) on pairs of meshes.
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

This will build two executables:
- `mesh-kernel`: The original mesh kernel computation tool
- `ember-csg`: The new EMBER CSG boolean operations tool

## Usage

### Mesh Kernel Computation

```bash
./mesh_kernel -i input_mesh.obj -o result_mesh.obj
```

### EMBER CSG Boolean Operations

```bash
./ember-csg -a mesh1.obj -b mesh2.obj -o result.obj --operation union
```

### Options

#### Mesh Kernel Options

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

#### EMBER CSG Options

| Flag                        | Description                                                                             |
| --------------------------- | --------------------------------------------------------------------------------------- |
| `-a, --input-a`             | Path to first input mesh (required)                                                     |
| `-b, --input-b`             | Path to second input mesh (required)                                                    |
| `-o, --output`              | Path to output mesh (required)                                                          |
| `--operation`               | CSG operation: `union`, `intersection`, `difference` (default: `union`)                 |
| `--format`                  | Output format: `obj`, `stl` (default: `obj`)                                            |
| `--no-exact`                | Disable exact arithmetic                                                                |
| `--no-validate`             | Disable result validation                                                               |

### Example

#### Mesh Kernel Computation

```bash
./mesh_kernel -i bunny.obj -o bunny_kernel.obj --triangulate
```

This command computes the kernel of the `bunny.obj` mesh, using Seidel's solver for early-out checks and saves the triangulated result as `bunny_kernel.obj`.

#### EMBER CSG Operations

```bash
# Union of two meshes
./ember-csg -a cube.obj -b sphere.obj -o union_result.obj --operation union

# Intersection of two meshes  
./ember-csg -a mesh1.stl -b mesh2.stl -o intersection.stl --operation intersection

# Difference of two meshes (A - B)
./ember-csg -a complex_mesh.obj -b simple_mesh.obj -o difference.obj --operation difference
```

<!-- ## License -->

<!-- [MIT](LICENSE) -->

<!-- --- -->
