# EMBER: Exact Mesh Boolean Operations

This document describes the EMBER (Exact Mesh Booleans via Efficient and Robust Local Arrangements) implementation in the mesh-kernel project.

## Overview

EMBER provides exact CSG (Constructive Solid Geometry) boolean operations on triangle meshes:
- **Union**: Combine two meshes, removing overlapping regions
- **Intersection**: Keep only the overlapping regions of two meshes  
- **Difference**: Subtract one mesh from another (A - B)

The implementation uses exact arithmetic and robust geometric algorithms to avoid numerical issues common in mesh boolean operations.

## Architecture

The EMBER system consists of several key components:

### 1. EmberCSG (`ember-csg.hh/cc`)
Main interface for CSG operations providing:
- High-level API for union, intersection, difference operations
- Coordinate conversion between double and exact integer arithmetic
- Integration with existing mesh-kernel infrastructure

### 2. MeshIntersector (`ember-intersect.hh/cc`) 
Computes intersections between meshes:
- BVH (Bounding Volume Hierarchy) spatial acceleration
- Triangle-triangle intersection tests with exact arithmetic
- Handling of coplanar triangles and degenerate cases
- Intersection curve construction

### 3. MeshClassifier (`ember-classify.hh/cc`)
Classifies mesh elements for CSG operations:
- Ray casting algorithm for inside/outside tests
- Multiple ray directions for robustness
- Boundary detection using intersection results
- Face-based classification using centroids

### 4. CSGConstructor (`ember-classify.hh/cc`)
Constructs final CSG results:
- Vertex mapping and face copying
- Operation-specific face selection
- Integration of intersection geometry
- Proper mesh topology construction

### 5. EmberApp (`ember-app.hh/cc`)
CLI application for EMBER operations:
- Command-line parsing and validation
- Mesh loading/saving in multiple formats
- Coordinate scaling and conversion
- Progress reporting and error handling

## Usage

### Command Line Interface

```bash
# Union of two meshes
./ember-csg -a cube.obj -b sphere.obj -o union.obj --operation union

# Intersection of two meshes
./ember-csg -a mesh1.stl -b mesh2.stl -o intersection.stl --operation intersection

# Difference (A minus B)
./ember-csg -a complex.obj -b simple.obj -o difference.obj --operation difference
```

### Command Line Options

| Option | Description |
|--------|-------------|
| `-a, --input-a PATH` | Path to first input mesh (required) |
| `-b, --input-b PATH` | Path to second input mesh (required) |
| `-o, --output PATH` | Path to output mesh (required) |
| `--operation OP` | CSG operation: union, intersection, difference |
| `--format FORMAT` | Output format: obj, stl |
| `--no-exact` | Disable exact arithmetic |
| `--no-validate` | Disable result validation |

### Programmatic API

```cpp
#include <core/ember-csg.hh>

// Create EMBER processor
mk::EmberCSG csg_processor;

// Load meshes and convert to integer coordinates
pm::Mesh mesh_a, mesh_b;
pm::vertex_attribute<mk::EmberCSG::pos_t> positions_a, positions_b;
// ... load and convert meshes ...

// Perform union operation
mk::ember_options options;
bool success = csg_processor.compute_union(positions_a, positions_b, options);

if (success) {
    auto& result_mesh = csg_processor.result_mesh();
    auto& result_positions = csg_processor.result_positions();
    // ... use result ...
}
```

## Algorithm Details

### Intersection Computation

1. **Spatial Acceleration**: BVH tree construction for efficient triangle pair queries
2. **Triangle-Triangle Tests**: Exact intersection computation using integer arithmetic
3. **Classification**: Vertex classification relative to triangle planes
4. **Coplanar Handling**: Special handling for coplanar triangle intersections
5. **Curve Construction**: Building connected intersection curves

### Element Classification

1. **Ray Casting**: Multiple rays from face centroids to determine inside/outside
2. **Intersection Counting**: Odd count = inside, even count = outside
3. **Boundary Detection**: Faces involved in intersections marked as boundary
4. **Robustness**: Multiple ray directions and exact arithmetic for reliability

### CSG Construction

1. **Face Selection**: Choose faces based on operation type and classification
2. **Vertex Mapping**: Map vertices from input meshes to result mesh
3. **Topology Construction**: Build proper mesh connectivity
4. **Intersection Integration**: Add geometry from intersection curves

## Exact Arithmetic

EMBER uses exact integer arithmetic throughout:
- Input coordinates scaled to integers (default: 1000x)
- All geometric computations use exact arithmetic
- No floating-point precision issues
- Robust handling of degenerate cases

### Coordinate Scaling

Input meshes are scaled from double precision to integer coordinates:
```cpp
// Automatic scaling based on coordinate range
double scale = get_scaling_factor(input_positions);

// Convert to integer positions
pos_t int_pos(
    static_cast<int64_t>(double_pos.x * scale),
    static_cast<int64_t>(double_pos.y * scale),
    static_cast<int64_t>(double_pos.z * scale)
);
```

## Testing

Run the EMBER test suite:
```bash
./ember-test
```

The test suite validates:
- CSG utility functions
- Basic CSG operations
- Mesh classification algorithms
- Integration between components

## Limitations

Current implementation limitations:
1. **Triangular Meshes Only**: Non-triangular faces are skipped
2. **Local Arrangements**: Full local arrangement construction not yet implemented
3. **Intersection Curves**: Basic intersection curve construction
4. **Performance**: Not yet optimized for very large meshes

## Future Enhancements

Planned improvements:
1. **Local Arrangements**: Complete local arrangement graph construction
2. **Non-Manifold Support**: Handle non-manifold mesh inputs
3. **Performance**: Multi-threading and SIMD optimizations
4. **Robustness**: Enhanced handling of edge cases
5. **Quality**: Mesh repair and quality improvement

## References

The EMBER implementation is based on:
- "Exact and Robust Mesh Arrangements for Computational Geometry"
- Robust geometric predicates literature
- Classical CSG algorithms adapted for exact arithmetic

## Integration with Mesh-Kernel

EMBER extends the existing mesh-kernel project by:
- Reusing integer-plane-geometry for exact arithmetic
- Following established code patterns and style
- Integrating with existing CLI infrastructure
- Supporting the same mesh formats and options