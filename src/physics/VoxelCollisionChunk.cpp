/**
 * VoxelCollisionChunk.cpp
 *
 * Debris-Voxel Collision System
 * Implementation of collision chunk with triangle mesh generation
 */

#include "VoxelCollisionChunk.h"
#include "VoxelPhysicsIntegration.h"  // For collision group enum
#include <iostream>

VoxelCollisionChunk::VoxelCollisionChunk() {
}

VoxelCollisionChunk::~VoxelCollisionChunk() {
    Cleanup();
}

void VoxelCollisionChunk::BuildFromVoxels(VoxelWorld& world, float chunk_size) {
#ifdef USE_BULLET
    // Clean up existing mesh
    if (triangle_mesh) {
        delete triangle_mesh;
        triangle_mesh = nullptr;
    }

    // Create new triangle mesh
    triangle_mesh = new btTriangleMesh();
    triangle_count = 0;

    // Calculate chunk bounds in world space
    Vector3 chunk_min(
        coord.x * chunk_size,
        coord.y * chunk_size,
        coord.z * chunk_size
    );

    Vector3 chunk_max(
        chunk_min.x + chunk_size,
        chunk_min.y + chunk_size,
        chunk_min.z + chunk_size
    );

    // Get all voxels in this chunk
    auto voxels = world.GetVoxelsInBox(chunk_min, chunk_max);

    if (voxels.empty()) {
        // No voxels in this chunk
        dirty = false;
        return;
    }

    // Add each voxel's exposed faces
    float voxel_size = world.GetVoxelSize();

    for (const auto& voxel_pos : voxels) {
        AddVoxelFacesToMesh(world, voxel_pos, voxel_size);
    }

    triangle_count = triangle_mesh->getNumTriangles();

    std::cout << "[VoxelCollisionChunk] Built chunk (" << coord.x << "," << coord.y << "," << coord.z
              << ") with " << triangle_count << " triangles\n";

    dirty = false;
#endif
}

void VoxelCollisionChunk::AddVoxelFacesToMesh(VoxelWorld& world, const Vector3& voxel_pos, float vs) {
#ifdef USE_BULLET
    // vs = voxel size (e.g., 0.05m)

    // Only add faces that are exposed (not touching another voxel)
    // This is a HUGE optimization - interior faces are skipped

    // Face directions
    Vector3 face_directions[6] = {
        Vector3( vs, 0,  0),   // +X
        Vector3(-vs, 0,  0),   // -X
        Vector3( 0,  vs, 0),   // +Y
        Vector3( 0, -vs, 0),   // -Y
        Vector3( 0,  0,  vs),  // +Z
        Vector3( 0,  0, -vs)   // -Z
    };

    // Check each face
    for (int face = 0; face < 6; face++) {
        Vector3 neighbor_pos = voxel_pos + face_directions[face];

        // Is there a voxel in this direction?
        if (world.HasVoxel(neighbor_pos)) {
            continue;  // Face is internal, skip it
        }

        // Face is exposed, add triangles
        AddFaceTriangles(face, voxel_pos, vs);
    }
#endif
}

void VoxelCollisionChunk::AddFaceTriangles(int face_index, const Vector3& pos, float vs) {
#ifdef USE_BULLET
    // Define cube vertices
    btVector3 vertices[8];
    vertices[0] = btVector3(pos.x,      pos.y,      pos.z);       // 0: ---
    vertices[1] = btVector3(pos.x + vs, pos.y,      pos.z);       // 1: +--
    vertices[2] = btVector3(pos.x + vs, pos.y + vs, pos.z);       // 2: ++-
    vertices[3] = btVector3(pos.x,      pos.y + vs, pos.z);       // 3: -+-
    vertices[4] = btVector3(pos.x,      pos.y,      pos.z + vs);  // 4: --+
    vertices[5] = btVector3(pos.x + vs, pos.y,      pos.z + vs);  // 5: +-+
    vertices[6] = btVector3(pos.x + vs, pos.y + vs, pos.z + vs);  // 6: +++
    vertices[7] = btVector3(pos.x,      pos.y + vs, pos.z + vs);  // 7: -++

    // Face vertex indices (CCW winding for outward normal)
    int face_indices[6][4] = {
        {1, 5, 6, 2},  // +X face (right)
        {4, 0, 3, 7},  // -X face (left)
        {3, 2, 6, 7},  // +Y face (top)
        {4, 5, 1, 0},  // -Y face (bottom)
        {5, 4, 7, 6},  // +Z face (front)
        {0, 1, 2, 3}   // -Z face (back)
    };

    int* indices = face_indices[face_index];

    // Add 2 triangles for this face
    // Triangle 1: 0-1-2
    triangle_mesh->addTriangle(
        vertices[indices[0]],
        vertices[indices[1]],
        vertices[indices[2]]
    );

    // Triangle 2: 0-2-3
    triangle_mesh->addTriangle(
        vertices[indices[0]],
        vertices[indices[2]],
        vertices[indices[3]]
    );
#endif
}

#ifdef USE_BULLET
void VoxelCollisionChunk::AddToPhysicsWorld(btDiscreteDynamicsWorld* world) {
    if (!triangle_mesh || triangle_count == 0) {
        return;  // Empty chunk, nothing to add
    }

    // DEBUG: Use simple box shape instead of triangle mesh to test collision
    std::cout << "[VoxelCollisionChunk] DEBUG: Creating btBoxShape instead of triangle mesh\n";
    btVector3 half_extents(0.025f, 0.25f, 0.025f);  // 0.05m x 0.5m x 0.05m box (tower dimensions)
    collision_shape = new btBoxShape(half_extents);
    collision_shape->setMargin(0.001f);

    std::cout << "[VoxelCollisionChunk] Created Box shape with half_extents=("
              << half_extents.x() << "," << half_extents.y() << "," << half_extents.z() << ")\n";

    // // Create BVH triangle mesh shape
    // // NOTE: useQuantizedAabbCompression = false for better accuracy
    // // buildBvh = true for faster queries
    // collision_shape = new btBvhTriangleMeshShape(
    //     triangle_mesh,
    //     false,  // useQuantizedAabbCompression - false for better accuracy with small voxels
    //     true    // buildBvh
    // );

    // // Set smaller collision margin for better accuracy with small voxels (default is 0.04m)
    // collision_shape->setMargin(0.001f);  // 1mm margin

    // std::cout << "[VoxelCollisionChunk] Created BVH shape with margin=" << collision_shape->getMargin() << "m\n";

    // // Get AABB to verify shape bounds
    // btVector3 aabb_min, aabb_max;
    // collision_shape->getAabb(btTransform::getIdentity(), aabb_min, aabb_max);
    // std::cout << "[VoxelCollisionChunk] Shape AABB: min=(" << aabb_min.x() << "," << aabb_min.y() << "," << aabb_min.z()
    //           << ") max=(" << aabb_max.x() << "," << aabb_max.y() << "," << aabb_max.z() << ")\n";

    // Create transform (offset to center the box at the tower location)
    btTransform transform;
    transform.setIdentity();
    // Box is centered at origin, so offset by half_extents to match tower position
    transform.setOrigin(btVector3(0.025f, 0.25f, 0.025f));
    std::cout << "[VoxelCollisionChunk] Transform origin=(" << transform.getOrigin().x() << ","
              << transform.getOrigin().y() << "," << transform.getOrigin().z() << ")\n";

    // Create motion state
    motion_state = new btDefaultMotionState(transform);

    // Create rigid body (mass = 0 for static)
    btRigidBody::btRigidBodyConstructionInfo info(
        0.0f,           // mass
        motion_state,
        collision_shape,
        btVector3(0, 0, 0)  // local inertia (unused for static)
    );

    rigid_body = new btRigidBody(info);

    // Mark as static object
    rigid_body->setCollisionFlags(
        rigid_body->getCollisionFlags() |
        btCollisionObject::CF_STATIC_OBJECT
    );
    std::cout << "[VoxelCollisionChunk] Set CF_STATIC_OBJECT flag, collision flags=0x"
              << std::hex << rigid_body->getCollisionFlags() << std::dec << "\n";

    // Set friction and restitution
    rigid_body->setFriction(0.9f);      // High friction
    rigid_body->setRestitution(0.1f);   // Low bounce

    // Set user data for identification
    rigid_body->setUserIndex(COL_VOXELS);
    rigid_body->setUserPointer(this);

    // DEBUG: Check world before/after adding body
    int num_before = world->getNumCollisionObjects();
    std::cout << "[VoxelCollisionChunk] Physics world has " << num_before << " objects before adding\n";

    // DEBUG: Temporarily add without collision filtering to test
    std::cout << "[VoxelCollisionChunk] DEBUG: Adding rigid body WITHOUT collision filtering\n";
    world->addRigidBody(rigid_body);

    int num_after = world->getNumCollisionObjects();
    std::cout << "[VoxelCollisionChunk] Physics world has " << num_after << " objects after adding (added=" << (num_after - num_before) << ")\n";

    // Force broadphase to recalculate after adding static body
    world->updateAabbs();
    world->getBroadphase()->calculateOverlappingPairs(world->getDispatcher());
    std::cout << "[VoxelCollisionChunk] Forced broadphase update\n";

    // // Add to physics world with collision filtering
    // short my_group = COL_VOXELS;
    // short my_mask = COL_DEBRIS | COL_UNITS | COL_PROJECTILES | COL_GROUND;

    // std::cout << "[VoxelCollisionChunk] Adding rigid body with collision filtering:\n";
    // std::cout << "  Group: 0x" << std::hex << my_group << std::dec << " (COL_VOXELS)\n";
    // std::cout << "  Mask:  0x" << std::hex << my_mask << std::dec
    //           << " (DEBRIS|UNITS|PROJECTILES|GROUND)\n";

    // world->addRigidBody(rigid_body, my_group, my_mask);
}

void VoxelCollisionChunk::RemoveFromPhysicsWorld(btDiscreteDynamicsWorld* world) {
    if (rigid_body) {
        world->removeRigidBody(rigid_body);

        delete rigid_body;
        rigid_body = nullptr;
    }

    if (motion_state) {
        delete motion_state;
        motion_state = nullptr;
    }

    if (collision_shape) {
        delete collision_shape;
        collision_shape = nullptr;
    }

    // Note: triangle_mesh is kept for potential rebuild
}
#endif

void VoxelCollisionChunk::Cleanup() {
#ifdef USE_BULLET
    if (triangle_mesh) {
        delete triangle_mesh;
        triangle_mesh = nullptr;
    }

    triangle_count = 0;
#endif
}
