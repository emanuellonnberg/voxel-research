/**
 * VoxelCollisionChunk.h
 *
 * Debris-Voxel Collision System
 * Individual collision chunk with triangle mesh
 */

#pragma once

#include "ChunkCoord.h"
#include "Vector3.h"
#include "VoxelWorld.h"

#ifdef USE_BULLET
#include <btBulletDynamicsCommon.h>
#endif

/**
 * A spatial chunk containing collision mesh for voxels
 *
 * Each chunk represents a region of space (e.g., 10m x 10m x 10m)
 * and contains a triangle mesh of all exposed voxel faces in that region.
 */
class VoxelCollisionChunk {
public:
    ChunkCoord coord;

    // Bullet physics objects
#ifdef USE_BULLET
    btRigidBody* rigid_body = nullptr;
    btTriangleMesh* triangle_mesh = nullptr;
    btBvhTriangleMeshShape* collision_shape = nullptr;
    btDefaultMotionState* motion_state = nullptr;
#endif

    // State
    bool dirty = true;
    int triangle_count = 0;

    VoxelCollisionChunk();
    ~VoxelCollisionChunk();

    /**
     * Build collision mesh from voxels in this chunk
     * @param world VoxelWorld to read voxels from
     * @param chunk_size Size of chunk in world units
     */
    void BuildFromVoxels(VoxelWorld& world, float chunk_size);

#ifdef USE_BULLET
    /**
     * Add this chunk's collision to physics world
     */
    void AddToPhysicsWorld(btDiscreteDynamicsWorld* world);

    /**
     * Remove this chunk's collision from physics world
     */
    void RemoveFromPhysicsWorld(btDiscreteDynamicsWorld* world);
#endif

    /**
     * Clean up all resources
     */
    void Cleanup();

private:
    /**
     * Add exposed faces of a voxel to the triangle mesh
     * @param world VoxelWorld to check neighbors
     * @param voxel_pos Position of voxel
     * @param voxel_size Size of one voxel
     */
    void AddVoxelFacesToMesh(VoxelWorld& world, const Vector3& voxel_pos, float voxel_size);

    /**
     * Add triangles for a specific face of a voxel cube
     * @param face_index Which face (0-5: +X, -X, +Y, -Y, +Z, -Z)
     * @param pos Position of voxel
     * @param voxel_size Size of voxel
     */
    void AddFaceTriangles(int face_index, const Vector3& pos, float voxel_size);
};
