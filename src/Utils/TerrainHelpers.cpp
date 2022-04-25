#include "TerrainHelpers.hpp"
#include <iostream>

namespace Terrains
{
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

    auto config = terrain.config;

    // add offset in meters (to compensate centering)
    auto xc = x + config.xSize/2;
    auto yc = y + config.ySize/2;

    // find grid coordinates for (xc, yc)
    int xg = xc / config.resolution;
    int yg = yc / config.resolution;

    auto numXsamples = (config.xSize/config.resolution) + 1;
    auto numYsamples = (config.ySize/config.resolution) + 1;

    int x1, x2, y1, y2;
    x1 = xg;
    y1 = yg;
    xg + 1 <= (numXsamples - 1) ?  x2 = xg + 1 : x2 = xg;
    yg + 1 <= (numYsamples - 1) ?  y2 = yg + 1 : y2 = yg;


    //Get heights at the corners
    float h00,h01,h10,h11;

    Eigen::Map<Eigen::Matrix<float, -1, -1>> hmap(terrain.heights.data(), numXsamples, numYsamples);
    h00 = hmap(x1, y1);
    h01 = hmap(x2, y1);
    h10 = hmap(x1, y2);
    h11 = hmap(x2, y2);


    // find tx, ty (computed in meters)
    float tx = ((x+ config.resolution) - x);
    float ty = ((y+ config.resolution) - y);

    std::cout << "XG , YG :"<<xg<<" , "<<yg<<std::endl;
    std::cout << "TX , TY :"<<tx<<" , "<<ty<<std::endl;
    return (1 - tx) * (1 - ty) * h00 +
           tx * (1 - ty) * h10 +
           (1 - tx) * ty * h01 +
           tx * ty * h11;
}

}