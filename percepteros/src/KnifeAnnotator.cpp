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

#include <cmath>

//surface matching
#include <pcl/console/print.h>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;

class KnifeAnnotator : public DrawingAnnotator
{
private:
	PCR::Ptr cloud_r = PCR::Ptr(new PCR);
	int GREEN_UPPER_BOUND = 80;
	int BLUE_LOWER_BOUND = 20;
	int RED_UPPER_BOUND = 80;
	int POINT_THRESHOLD = 10;
	float MAX_DISTANCE = 0.2f;

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
			std::vector<float> orientation = checkCluster(cluster, cloud_r);
			
			if (orientation[0] != 0 && orientation[1] != 0 && orientation[2] != 0) {
				outInfo("found knife");
				found = true;

				PointR highest = getHandle(cluster, cloud_r, orientation);

				//can't give tcas to function, so have to do it here
				percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);
				tf::Transform transform = publishResults(cluster, highest, orientation, o);
				
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

	std::vector<float> checkCluster(rs::Cluster cluster, PCR::Ptr cloud_ptr) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);
		int colorCounter = 0;
		std::vector<float> orientation(3);
		PointR temp;
		outInfo("Cluster size: " << cluster_indices->indices.size());

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
				 pit != cluster_indices->indices.end(); pit++) {
			temp = cloud_ptr->points[*pit];
			if ((int) temp.g > GREEN_UPPER_BOUND && (int) temp.b < BLUE_LOWER_BOUND && (int) temp.r > RED_UPPER_BOUND) {
				colorCounter++;
				orientation[0] += temp.x;
				orientation[1] += temp.y;
				orientation[2] += temp.z;
				//outInfo("RGB: " << (int) temp.r << "/" << (int) temp.g << "/" << (int) temp.b);
			}
		}
		
		if (colorCounter > POINT_THRESHOLD) {
			orientation[0] /= colorCounter;
			orientation[1] /= colorCounter;
			orientation[2] /= colorCounter;
		} else {
			orientation[0] = 0;
			orientation[1] = 0;
			orientation[2] = 0;
		}

		return orientation;
	}

	PointR getHandle(rs::Cluster cluster, PCR::Ptr cloud_ptr, std::vector<float> orientation) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);
		PointR temp;
		PointR highest = cloud_ptr->points[0];
		PointR dist;
		dist.x = orientation[0];
		dist.y = orientation[1];
		dist.z = orientation[2];

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
				 pit != cluster_indices->indices.end(); pit++) {
			temp = cloud_ptr->points[*pit];
			if ((int) temp.x > (int) highest.x && pcl::geometry::distance(temp, dist) < MAX_DISTANCE) {
				highest = temp;
			}
		}

		return highest;
	}

	tf::Transform publishResults(rs::Cluster cluster, PointR highest, std::vector<float> orientation, percepteros::RecognitionObject o) {
		tf::Transform transform;
		
		tf::Vector3 trans(highest.x, highest.y, highest.z);		
		transform.setOrigin(trans);
		
		//see http://math.stackexchange.com/a/476311
		tf::Vector3 up(highest.x, highest.y, highest.z);
		tf::Vector3 down(orientation[0], orientation[1], orientation[2]);
		up.normalize();
		down.normalize();
	
		tf::Vector3 cross = up.cross(down);
		float cos = 1 / (1 + up.dot(down));
		
		tf::Matrix3x3 vx;
		vx.setValue(0, -cross[2], cross[1],
								cross[2], 0, -cross[0],
								-cross[1], cross[0], 0);
		tf::Matrix3x3 vx2 = (vx * vx); 
		
		tf::Matrix3x3 rot;
		rot.setValue(	1, vx[0][1] + vx2[0][1] * cos, vx[0][2] + vx2[0][2] * cos, 
									vx[1][0] + vx2[1][0] * cos,	1, vx[1][2] + vx2[1][2] * cos, 
									vx[2][0] + vx2[2][0] * cos, vx[2][0] + vx2[2][0] * cos, 1);

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
