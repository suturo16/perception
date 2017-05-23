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
#include <pcl/filters/voxel_grid.h>

#include <cmath>

//surface matching
#include <pcl/console/print.h>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::Normal	Normal;
typedef pcl::PointCloud<Normal> PCN;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN> PC;

class SpatulAnnotator : public DrawingAnnotator
{
private:
	PCR::Ptr cloud_r = PCR::Ptr(new PCR);
	PCN::Ptr cloud_n = PCN::Ptr(new PCN);
	PC::Ptr cloud = PC::Ptr(new PC);
	PC::Ptr blade = PC::Ptr(new PC);
	PC::Ptr rack = PC::Ptr(new PC);

public:
	tf::Vector3 x, y, z;
	PointN highest, lowest;

  SpatulAnnotator(): DrawingAnnotator(__func__){
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

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
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud_r, *temp);
		pcl::concatenateFields(*temp, *cloud_n, *cloud);

		//helpers
		rs::StopWatch clock;

		std::vector<percepteros::RecognitionObject> objects;
		for (auto it = clusters.begin(); it != clusters.end(); ++it) {
			auto cluster = *it;
			if (cluster.source.get().compare("HueClustering") == 0) {
				objects.clear();
				cluster.annotations.filter(objects);
				
				outInfo("Average hue: " << objects[0].color.get());
				pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
				rs::ReferenceClusterPoints clusterpoints(cluster.points());
				rs::conversion::from(clusterpoints.indices(), *cluster_indices);
			} 	
		}
		
   		return UIMA_ERR_NONE;
  }

	void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
	}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SpatulAnnotator)
