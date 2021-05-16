#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    //root = recursiveBuild(primitives);
    root = recursiveBuildSAH(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*> objects){
    BVHBuildNode *node = new BVHBuildNode();

    // compute bounds of all primitives in SVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); i++){
        bounds = Union(bounds, objects[i]->getBounds());
    }
    if(objects.size() == 1){
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if(objects.size() == 2){
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else{
        Bounds3 centroidBounds;
        constexpr int nBunckets = 12;
        float minCost = std::numeric_limits<float>::max();
        int minCostSplitBucket = 0;

        // 记录按最小代价划分的索引
        int mid;
        // 记录按最小代价划分对应的维度
        int minDim = 0;

        int start = 0;
        int end = objects.size();

        for (int i = 0; i < end;i++){
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
        }

        // 对三个维度分别计算最小代价
        for (int dim = 0; dim < 3; dim++){
            //如果图元数量小于4，直接取中间即可，减少运算时间
            if(objects.size() <= 4){
                mid = (start + end) / 2;
                std::nth_element(&objects[start], &objects[mid], &objects[end - 1] + 1,
                    [dim](Object *a, Object *b) {
                    return a->getBounds().Centroid()[dim] < b->getBounds().Centroid()[dim];
                    });
            }
            else{
                // 否则，按最小代价进行划分，首先初始化格子
                BucketInfo buckets[nBunckets];
                for (int i = 0; i < end; ++i){
                    Vector3f offsetVec = centroidBounds.Offset(objects[i]->getBounds().Centroid());
                    int b = nBunckets * offsetVec[dim];
                    if(b == nBunckets)
                        b = nBunckets - 1;
                    buckets[b].count++;
                    buckets[b].bounds = Union(buckets[b].bounds, objects[i]->getBounds().Centroid());
                }

                //计算最小代价
                float cost[nBunckets - 1];
                for (int i = 0; i < nBunckets - 1; ++i){
                    Bounds3 b0, b1;
                    int count0 = 0, count1 = 0;
                    for (int j = 0; j <= i; j++){
                        b0 = Union(b0, buckets[j].bounds);
                        count0 += buckets[j].count;
                    }
                    for (int j = i + 1; j < nBunckets; ++j){
                        b1 = Union(b1, buckets[j].bounds);
                        count1 += buckets[j].count;
                    }
                    cost[i] = .125f + (count0 * b0.SurfaceArea() + count1 * b1.SurfaceArea()) / bounds.SurfaceArea();

                    //更新最小代价参数：最小代价，最小代价的格子索引，最小代价的维度
                    if(cost[i] < minCost){
                        minCost = cost[i];
                        minCostSplitBucket = i;
                        minDim = dim;
                    }
                }
            }
        }

        //按最小代价进行划分
        auto pmid = std::partition(&objects[0], &objects[end - 1] + 1,
            [=](Object *pi) {
                int b = nBunckets * centroidBounds.Offset(pi->getBounds().Centroid())[minDim];
                if (b == nBunckets) b = nBunckets - 1;
                return b <= minCostSplitBucket;
            });
        mid = pmid - &objects[0];

        auto beginning = objects.begin();
        auto middling = objects.begin() + mid;
        auto ending = objects.end();

        auto leftshapes = std::vector<Object *>(beginning, middling);
        auto rightshapes = std::vector<Object *>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuildSAH(leftshapes);
        node->right = recursiveBuildSAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;
    
    //光线方向
    float x = ray.direction.x;
    float y = ray.direction.y;
    float z = ray.direction.z;
    //判断光线方向是否为负
    std::array<int, 3> dirsIsNeg{int(x > 0), int(y > 0), int(z > 0)};
    
    //判断节点的包围盒与光线是否相交
    if(node->bounds.IntersectP(ray, ray.direction_inv, dirsIsNeg) == false)
        return inter;
    
    if(node->left == nullptr && node->right == nullptr)
    {
        inter = node->object->getIntersection(ray);
        return inter;
    }

    //递归判断
    auto hit1 = getIntersection(node->left, ray);
    auto hit2 = getIntersection(node->right, ray);

    if(hit1.distance < hit2.distance)
        return hit1;
    return hit2;
}