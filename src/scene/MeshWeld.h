#pragma once
#include <cstdint>
#include <vector>

// Indexed form of a MeshGeometry triangle soup. Vertices that are bitwise
// identical (position and normal) are merged, so the conversion is lossless:
// unweld(weld(soup)) == soup, and welding the same soup always gives the
// same result (vertices in order of first use).
struct WeldedMesh {
    std::vector<float>    vertices;   // pos.xyz + normal.xyz, 6 floats per vertex
    std::vector<uint32_t> indices;    // 3 per triangle
};

WeldedMesh         weldMesh  (const std::vector<float>& interleaved);
std::vector<float> unweldMesh(const std::vector<float>& vertices, const std::vector<uint32_t>& indices);
