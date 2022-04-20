#ifndef DARTROBOTS_TERRAINHELPERS_H
#define DARTROBOTS_TERRAINHELPERS_H
#include <dart/dart.hpp>

namespace {
dart::dynamics::SkeletonPtr DartTerrainFromData(Terrain terrain)
{

    auto config = terrain.config;
    // create shape of terrrain
    auto terrainShape = std::make_shared<dart::dynamics::HeightmapShape<float>>();
    auto scale = Eigen::Vector3f{config.resolution, config.resolution, 1.0};

    auto xs = int(config.xSize / config.resolution) + 1;
    auto ys = int(config.ySize / config.resolution) + 1;

    terrainShape->setHeightField(xs, ys, terrain.heights);
    terrainShape->setScale(scale);
    // Make skeleton
    dart::dynamics::SkeletonPtr terrainSkel = dart::dynamics::Skeleton::create();

    dart::dynamics::BodyNodePtr terrainBodyCollision =
        terrainSkel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;

    Eigen::Isometry3d tf_trans = Eigen::Isometry3d::Identity();
    tf_trans.translation() = Eigen::Vector3d{0.0, 0.0, 0.0};
    terrainBodyCollision->getParentJoint()->setTransformFromParentBodyNode(tf_trans);

    dart::dynamics::ShapeNode *shapeNodeCollision =
        terrainBodyCollision->createShapeNodeWith<dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(
            terrainShape);

    shapeNodeCollision->setRelativeTransform(tf_trans);

    dart::dynamics::BodyNodePtr terrainBodyVisual =
        terrainSkel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;

    Eigen::Isometry3d tf_trans_vis = Eigen::Isometry3d::Identity();
    tf_trans_vis.translation() = Eigen::Vector3d{-config.xSize / 4.0, config.ySize / 4.0, 0.0};
    //    tf_trans_vis.translation() = Eigen::Vector3d{0.0,0.0,0.0};
    terrainBodyVisual->getParentJoint()->setTransformFromParentBodyNode(tf_trans_vis);

    dart::dynamics::ShapeNode *shapeNodeVisual =
        terrainBodyVisual->createShapeNodeWith<dart::dynamics::VisualAspect>(terrainShape);
    shapeNodeVisual->setRelativeTransform(tf_trans_vis);

    return terrainSkel;
}

float GetHeight(float x, float y, Terrain &terrain)
{
    /*
     C10----------C11
     |            |
     |            |
     |            |
     |            |
     C00----------C01
     */

    // Bi-linear interpolation

    // find the leftmost corner of square that contains (x,y)
    int xi = std::floor(x);
    int yi = std::floor(y);

    auto xs = (terrain.config.xSize / terrain.config.resolution) + 1;
    auto ys = (terrain.config.ySize / terrain.config.resolution) + 1;

    //Get heights at the corners
    Eigen::Map<Eigen::Matrix<float, -1, -1>> hmap(terrain.heights.data(), xs, ys);
    auto h00 = hmap(xi, yi);
    auto h01 = hmap(xi + 1, yi);
    auto h10 = hmap(xi, yi + 1);
    auto h11 = hmap(xi + 1, yi + 1);

    // find tx, ty
    float tx = x - xi;
    float ty = y - yi;

    return (1 - tx) * (1 - ty) * h00 +
           tx * (1 - ty) * h10 +
           (1 - tx) * ty * h01 +
           tx * ty * h11;
}

}

#endif // DARTROBOTS_TERRAINHELPERS_H
