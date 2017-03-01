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

class KnifeAnnotator : public DrawingAnnotator
{
private:
	PCR::Ptr cloud_r = PCR::Ptr(new PCR);
	PCN::Ptr cloud_n = PCN::Ptr(new PCN);
	float GREEN_UPPER_BOUND, BLUE_LOWER_BOUND, RED_UPPER_BOUND, POINT_THRESHOLD, MAX_DISTANCE;

public:

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

		//helpers
		rs::StopWatch clock;
		bool found = false;

		for (auto cluster : clusters) {
			std::vector<float> orientation = checkCluster(cluster, cloud_r, cloud_n);
			
			if (orientation[0] != 0 && orientation[1] != 0 && orientation[2] != 0
					&& orientation[3] != 0 && orientation[4] != 0 && orientation[5] != 0) {
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

	std::vector<float> checkCluster(rs::Cluster cluster, PCR::Ptr cloud_ptr, PCN::Ptr normal_ptr) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);
		int colorCounter = 0;
		int normalCounter = 0;
		std::vector<float> orientation(6);
		PointR temp;
		PointN tempN;
		//outInfo("Cluster size: " << cluster_indices->indices.size());

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
				 pit != cluster_indices->indices.end(); pit++) {
			temp = cloud_ptr->points[*pit];
			tempN = normal_ptr->points[*pit];
			if ((int) temp.g > GREEN_UPPER_BOUND && (int) temp.b < BLUE_LOWER_BOUND && (int) temp.r > RED_UPPER_BOUND) {
				colorCounter++;
				//middle value for location
				orientation[0] += temp.x;
				orientation[1] += temp.y;
				orientation[2] += temp.z;
				//middle value for normal
				if (tempN.normal_x == tempN.normal_x &&
						tempN.normal_y == tempN.normal_y &&
						tempN.normal_z == tempN.normal_z) {
					orientation[3] += tempN.normal_x;
					orientation[4] += tempN.normal_y;
					orientation[5] += tempN.normal_z;
					normalCounter++;
				}
				//outInfo("RGB: " << (int) temp.r << "/" << (int) temp.g << "/" << (int) temp.b);
			}
		}
		
		if (colorCounter > POINT_THRESHOLD) {
			//middle value for location
			orientation[0] /= colorCounter;
			orientation[1] /= colorCounter;
			orientation[2] /= colorCounter;
			//middle value for normal
			orientation[3] /= normalCounter;
			orientation[4] /= normalCounter;
			orientation[5] /= normalCounter;
		} else {
			orientation[0] = 0;
			orientation[1] = 0;
			orientation[2] = 0;
			orientation[3] = 0;
			orientation[4] = 0;
			orientation[5] = 0;
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
			if ((int) temp.x > (int) highest.x && pcl::geometry::distance(temp, dist) < MAX_DISTANCE)	{
				highest = temp;
			}
		}

		return highest;
	}

	tf::Transform publishResults(rs::Cluster cluster, PointR highest, std::vector<float> orientation, percepteros::RecognitionObject o) {
		tf::Transform transform;
		
		tf::Vector3 trans(highest.x, highest.y, highest.z);		
		transform.setOrigin(trans);
		
		//outInfo("Median normal: " << orientation[3] << "," << orientation[4] << "," << orientation[5]);

		tf::Vector3 z(orientation[3], orientation[4], orientation[5]);
		tf::Vector3 x(orientation[0] - highest.x, orientation[1] - highest.y, orientation[2] - highest.z);
		tf::Vector3 y = z.cross(x);
		z = x.cross(y);

		x.normalize();
		y.normalize();
		z.normalize();

		tf::Matrix3x3 rot;
		rot.setValue(x[0], x[1], x[2],
								 y[0], y[1], y[2],
								 z[0], z[1], z[2]);
		transform.setBasis(rot);

		o.name.set("Knife");
		o.type.set(6);
		o.width.set(0.056f);
		o.height.set(0.28f);
		o.depth.set(0.03f);
		
		return transform;
	}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(KnifeAnnotator)
