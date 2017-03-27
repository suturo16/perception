#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/common.h>
#include <rs/utils/time.h>

#include <percepteros/types/all_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/geometry.h>
#include <iostream>

#include <cmath>

//surface matching
#include <pcl/console/print.h>

using namespace uima;

typedef pcl::Normal	PointN;
typedef pcl::PointCloud<PointN> PCN;
typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::PointXYZHSV PointH;
typedef pcl::PointCloud<PointH> PCH;

class KnifeAnnotator : public DrawingAnnotator
{
private:
	PCR::Ptr cloud_r = PCR::Ptr(new PCR);
	PCN::Ptr cloud_n = PCN::Ptr(new PCN);
	PCH::Ptr cloud_h = PCH::Ptr(new PCH);
	PCR::Ptr blade = PCR::Ptr(new PCR);
	PCN::Ptr blade_n = PCN::Ptr(new PCN);
	float GREEN_UPPER_BOUND, BLUE_LOWER_BOUND, RED_UPPER_BOUND, POINT_THRESHOLD, MAX_DISTANCE;

public:
	tf::Vector3 x, y, z;
	PointR highest, lowest;

  KnifeAnnotator(): DrawingAnnotator(__func__){
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
		
		//extract color parameters
		ctx.extractValue("maxGreen", GREEN_UPPER_BOUND);
		ctx.extractValue("minBlue", BLUE_LOWER_BOUND);
		ctx.extractValue("maxRed", RED_UPPER_BOUND);

		//extract threshold
		ctx.extractValue("minPoints", POINT_THRESHOLD);
		ctx.extractValue("maxDistance", MAX_DISTANCE);

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start\n");
		//get clusters
    rs::SceneCas cas(tcas);
		rs::Scene scene = cas.getScene();
		std::vector<rs::Cluster> clusters;
		scene.identifiables.filter(clusters);

		//get scene points
		cas.get(VIEW_CLOUD, *cloud_r);
		cas.get(VIEW_NORMALS, *cloud_n);
		pcl::PointCloudXYZRGBAtoXYZHSV(*cloud_r, *cloud_h);

		//helpers
		rs::StopWatch clock;
		bool found = false;

		for (auto cluster : clusters) {
			blade = PCR::Ptr(new PCR);
			blade_n = PCN::Ptr(new PCN);
			found = checkCluster(cluster, cloud_r, cloud_h, blade, blade_n);
			
		}
    return UIMA_ERR_NONE;
  }

	bool checkCluster(rs::Cluster cluster, PCR::Ptr cloud_ptr, PCH::Ptr hsv_ptr, PCR::Ptr blade, PCN::Ptr blade_n) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);
		PointH temp;
		float av = 0;
		int size = cluster_indices->indices.size();

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
				 pit != cluster_indices->indices.end(); pit++) {
			temp = hsv_ptr->points[*pit];
			av += temp.h / size;
		}
		outInfo("Size: " << size << " H: " << av);
		return true;
	}

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(KnifeAnnotator)
