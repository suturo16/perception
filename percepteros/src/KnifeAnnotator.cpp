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

		//helpers
		rs::StopWatch clock;
		bool found = false;

		for (auto cluster : clusters) {
			blade = PCR::Ptr(new PCR);
			blade_n = PCN::Ptr(new PCN);
			found = checkCluster(cluster, cloud_r, cloud_n, blade, blade_n);
			
			if (found) {
				outInfo("found knife");

				std::vector<PointR> endpoints = getX(blade);
				y = getY(blade_n);

				if (endpoints[0].x + endpoints[0].y + endpoints[0].z <
						endpoints[1].x + endpoints[1].y + endpoints[1].z) {
					highest = endpoints[0];
					lowest = endpoints[1];
				} else {
					highest = endpoints[1];
					lowest = endpoints[0];
				}
				x.setValue(lowest.x - highest.x, lowest.y - highest.y, lowest.z - highest.z);

				percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);
				tf::Transform transform;
		
				tf::Vector3 trans(highest.x, highest.y, highest.z);		
				transform.setOrigin(trans);
				
				z = x.cross(y);
				y = z.cross(x);

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
				o.width.set(0.28f);
				o.height.set(0.056f);
				o.depth.set(0.03f);

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
				poseAnnotation.source.set("3DEstimate");

				cluster.annotations.append(o);
				cluster.annotations.append(poseAnnotation);

				break;
			}
			outInfo("Finished recognition in: " << clock.getTime() << "ms.");
		}
		
		if (!found) {
			outInfo("No matching cluster");
		}

    return UIMA_ERR_NONE;
  }

	bool checkCluster(rs::Cluster cluster, PCR::Ptr cloud_ptr, PCN::Ptr normal_ptr, PCR::Ptr blade, PCN::Ptr blade_n) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);
		PointR temp;
		PointN tempN;

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
				 pit != cluster_indices->indices.end(); pit++) {
			temp = cloud_ptr->points[*pit];
			tempN = normal_ptr->points[*pit];
			if ((int) temp.g > GREEN_UPPER_BOUND && (int) temp.b < BLUE_LOWER_BOUND && (int) temp.r > RED_UPPER_BOUND) {
				blade->push_back(temp);
				//middle value for normal
				if (tempN.normal_x == tempN.normal_x &&
						tempN.normal_y == tempN.normal_y &&
						tempN.normal_z == tempN.normal_z) {
					blade_n->push_back(tempN);
				}
				//outInfo("RGB: " << (int) temp.r << "/" << (int) temp.g << "/" << (int) temp.b);
			}
		}
		
		if (blade->size() > POINT_THRESHOLD) {
			return true;
		} else {
			return false;
		}
	}

	std::vector<PointR> getX(PCR::Ptr blade) {
		PointR begin, end;
		std::vector<PointR> endpoints(2);
		float currDistance = 0;
		int size = blade->size();

		for (int i = 0; i < size; i++) {
			begin = blade->points[i];
			for (int j = 0; j < size; j++) {
				end = blade->points[j];
				if (pcl::geometry::distance(begin, end) > currDistance) {
					endpoints[0] = begin;
					endpoints[1] = end;
					currDistance = pcl::geometry::distance(begin, end);
				}
			}
		}
		
		return endpoints;
	}

	tf::Vector3 getY(PCN::Ptr blade_n) {
		tf::Vector3 blade_normal(0, 0, 0);
		int size = blade_n->size();

		for (int i = 0; i < size; i++) {
			blade_normal[0] -= blade_n->points[i].normal_x / size;
			blade_normal[1] -= blade_n->points[i].normal_y / size;
			blade_normal[2] -= blade_n->points[i].normal_z / size;
		}

		return blade_normal;
	}

	void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
		if (firstRun) {
			visualizer.addPointCloud(cloud_r, "Scene Points");
		} else {
			visualizer.updatePointCloud(cloud_r, "Scene Points");
		}

		visualizer.addCone(getCoefficients(x, highest), "x");
		visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
																					 1, 0, 0, "x");
		visualizer.addCone(getCoefficients(y, highest), "y");
		visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
																					 0, 1, 0, "y");
		visualizer.addCone(getCoefficients(z, highest), "z");
		visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 
																					 0, 0, 1, "z");
	}

	pcl::ModelCoefficients getCoefficients(tf::Vector3 axis, PointR highest) {
		pcl::ModelCoefficients coeffs;
		//point
		coeffs.values.push_back(highest.x);
		coeffs.values.push_back(highest.y);
		coeffs.values.push_back(highest.z);
		//direction
		coeffs.values.push_back(axis[0]);
		coeffs.values.push_back(axis[1]);
		coeffs.values.push_back(axis[2]);
		//radius
		coeffs.values.push_back(1.0f);	

		return coeffs;
	}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(KnifeAnnotator)
