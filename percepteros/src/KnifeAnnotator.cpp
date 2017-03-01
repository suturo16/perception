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

//surface matching
#include <pcl/console/print.h>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;

class KnifeAnnotator : public DrawingAnnotator
{
private:
	PCR::Ptr cloud_r = PCR::Ptr(new PCR);
	float GREEN_UPPER_BOUND = 140;
	float BLUE_LOWER_BOUND = 20;
	float RED_UPPER_BOUND = 150;
	int POINT_THRESHOLD = 50;

public:

  KnifeAnnotator(): DrawingAnnotator(__func__){
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
		
		//helpers
		rs::StopWatch clock;
		bool found = false;

		for (auto cluster : clusters) {
			found = checkCluster(cluster, cloud_r);
			
			if (found) {
				outInfo("found knife");
				
				PointR highest = getHandle(cluster, cloud_r);

				//can't give tcas to function, so have to do it here
				percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);
				tf::Transform transform = publishResults(cluster, highest, o);
				
				rs::PoseAnnotation poseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
				tf::StampedTransform camToWorld;
				camToWorld.setIdentity();
				if (scene.viewPoint.has()) {
					rs::conversion::from(scene.viewPoint.get(), camToWorld);
				}

				tf::Stamped<tf::Pose> camera(transform, camToWorld.stamp_, camToWorld.child_frame_id_);
				tf::Stamped<tf::Pose> world(camToWorld * transform, camToWorld.stamp_, camToWorld.frame_id_);

				poseAnnotation.camera.set(rs::conversion::to(tcas, camera));
				poseAnnotation.world.set(rs::conversion::to(tcas, world));
				poseAnnotation.source.set("Estimate");

				cluster.annotations.append(o);
				cluster.annotations.append(poseAnnotation);
				//scene.identifiables.append(cluster);
				break;
			}
			outInfo("Finished recognition in: " << clock.getTime() << "ms.");
		}
		
		if (!found) {
			outInfo("No matching cluster");
		}

    return UIMA_ERR_NONE;
  }

	bool checkCluster(rs::Cluster cluster, PCR::Ptr cloud_ptr) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);
		int colorCounter = 0;
		PointR temp;
		outInfo("Cluster size: " << cluster_indices->indices.size());

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
				 pit != cluster_indices->indices.end(); pit++) {
			temp = cloud_ptr->points[*pit];
			if ((int) temp.g > GREEN_UPPER_BOUND && (int) temp.b < BLUE_LOWER_BOUND && (int) temp.r > RED_UPPER_BOUND) {
				colorCounter++;
			}
		}
		
		if (colorCounter > POINT_THRESHOLD) {
			return true;
		} else {
			return false;
		}
	}

	PointR getHandle(rs::Cluster cluster, PCR::Ptr cloud_ptr) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);
		PointR temp;
		PointR highest = cloud_ptr->points[0];

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
				 pit != cluster_indices->indices.end(); pit++) {
			temp = cloud_ptr->points[*pit];
			if ((int) temp.x > (int) highest.x) {
				highest = temp;
			}
		}

		return highest;
	}

	tf::Transform publishResults(rs::Cluster cluster, PointR highest, percepteros::RecognitionObject o) {
		tf::Transform transform;
		
		tf::Vector3 trans(highest.x, highest.y, highest.z);
		tf::Matrix3x3 rot;
		rot.setValue(1, 0, 0, 0, 1, 0, 0, 0, 1);

		transform.setOrigin(trans);
		transform.setBasis(rot);

		o.name.set("Knife");
		o.type.set(6);
		o.width.set(0);
		o.height.set(0);
		o.depth.set(0);
		
		return transform;
	}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(KnifeAnnotator)
