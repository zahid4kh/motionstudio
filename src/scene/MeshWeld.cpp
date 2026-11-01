#include "MeshWeld.h"
#include <array>
#include <cstring>
#include <unordered_map>

namespace {
using VertexKey = std::array<uint32_t, 6>;   // the six floats' bit patterns

struct VertexKeyHash {
    size_t operator()(const VertexKey& k) const {
        uint64_t h = 1469598103934665603ull;   // FNV-1a over the words
        for (uint32_t w : k) { h ^= w; h *= 1099511628211ull; }
        return size_t(h);
    }
};
}

WeldedMesh weldMesh(const std::vector<float>& interleaved) {
    WeldedMesh out;
    const size_t count = interleaved.size() / 6;
    out.indices.reserve(count);
    std::unordered_map<VertexKey, uint32_t, VertexKeyHash> seen;
    seen.reserve(count / 2);
    for (size_t i = 0; i < count; ++i) {
        const float* v = &interleaved[i * 6];
        VertexKey key;
        std::memcpy(key.data(), v, sizeof(key));
        auto [it, added] = seen.try_emplace(key, uint32_t(out.vertices.size() / 6));
        if (added) out.vertices.insert(out.vertices.end(), v, v + 6);
        out.indices.push_back(it->second);
    }
    return out;
}

std::vector<float> unweldMesh(const std::vector<float>& vertices, const std::vector<uint32_t>& indices) {
    std::vector<float> out;
    const size_t vertexCount = vertices.size() / 6;
    out.reserve(indices.size() * 6);
    for (uint32_t i : indices) {
        if (i >= vertexCount) return {};   // corrupt data
        out.insert(out.end(), &vertices[size_t(i) * 6], &vertices[size_t(i) * 6] + 6);
    }
    return out;
}
