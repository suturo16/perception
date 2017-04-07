/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author(s): Ferenc Balint-Benczedi <balintbe@cs.uni-bremen.de>
 *         Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *         Jan-Hendrik Worch <jworch@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// UIMA
#include <uima/api.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>
#include <rs/utils/output.h>

// PCL
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/pca.h>

// OTHER
#include <percepteros/types/all_types.h>
#include <rs/segmentation/ImageSegmentation.h>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;

class TrayAnnotator : public DrawingAnnotator
{

private:
	PCR::Ptr cloud = PCR::Ptr(new PCR);
	PCR::Ptr tray = PCR::Ptr(new PCR);
	PCR::Ptr proj = PCR::Ptr(new PCR);

	tf::Matrix3x3 rot;
	tf::Vector3 origin;

public:
  TrayAnnotator() : DrawingAnnotator(__func__)
  {
  }

  /*
   * Initializes annotator
   */
  TyErrorId initialize(AnnotatorContext &ctx)
  {
    return UIMA_ERR_NONE;
  }

  /*
   * Destroys annotator
   */
  TyErrorId destroy()
  {
    return UIMA_ERR_NONE;
  }

private:
  /*
   * Processes a frame
   */
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    cas.get(VIEW_CLOUD, *cloud);

		std::vector<rs::Cluster> clusters;
		scene.identifiables.filter(clusters);

		pcl::SACSegmentation<PointR> seg;
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.01);
	
		pcl::PCA<PointR> pca; 

		for (auto cluster : clusters) {
			if (cluster.source.get().compare(0, 13, "HueClustering") == 0) {
				pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
				rs::ReferenceClusterPoints clusterpoints(cluster.points());
				rs::conversion::from(clusterpoints.indices(), *cluster_indices);

				extractPoints(cloud, tray, cluster_indices);

				pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
				pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

				seg.setInputCloud(tray);
				seg.segment(*inliers, *coefficients);
				
				if (inliers->indices.size() / tray->points.size() > 0.9) {
					float hue = std::strtof(cluster.source.get().substr(15).data(), NULL);

					if (hue < 10 || hue > 320 || (hue > 40 && hue < 80)) {
						pca.setInputCloud(tray);

						auto ev = pca.getEigenVectors();

						percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);
						rs::PoseAnnotation poseA = rs::create<rs::PoseAnnotation>(tcas);
						tf::StampedTransform camToWorld;
						if (scene.viewPoint.has()) {
							rs::conversion::from(scene.viewPoint.get(), camToWorld);
						}

						tf::Transform transform;
						origin = getOrigin(tray);
						transform.setOrigin(getOrigin(tray));

						rot.setValue(ev(0,0), ev(0,1), ev(0,2), ev(1,0), ev(1,1), ev(1,2), ev(2,0), ev(2,1), ev(2,2));
						transform.setBasis(rot);

						o.name.set("DropZone");
						o.type.set(5);
						o.width.set(0.21);
						o.height.set(0.29);
						o.depth.set(0);
						
						tf::Stamped<tf::Pose> camera(transform, camToWorld.stamp_, camToWorld.child_frame_id_);
						tf::Stamped<tf::Pose> world(camToWorld * transform, camToWorld.stamp_, camToWorld.frame_id_);

						poseA.source.set("TrayAnnotator");
						poseA.camera.set(rs::conversion::to(tcas, camera));
						poseA.world.set(rs::conversion::to(tcas, world));

						cluster.annotations.append(poseA);
						cluster.annotations.append(o);
					}
				}
			}
		}
		return UIMA_ERR_NONE;
  }

	void extractPoints(PCR::Ptr cloud, PCR::Ptr object, pcl::PointIndices::Ptr indices) {
		object->clear();
		for (size_t i = 0; i < indices->indices.size(); i++) {
			object->push_back(cloud->points[indices->indices[i]]);
		}
	}

	tf::Vector3 getOrigin(PCR::Ptr tray) {
		tf::Vector3 origin(0, 0, 0);
		int size = tray->points.size();

		for (int i = 0; i < tray->points.size(); i++) {
			origin[0] += tray->points[i].x / size;
			origin[1] += tray->points[i].y / size;
			origin[2] += tray->points[i].z / size;
		}

		return origin;
	}

	void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &vis, const bool firstRun) {
		if (firstRun) {
			vis.addPointCloud(cloud, "scene points");
		} else {
			vis.updatePointCloud(cloud, "scene points");
			vis.removeAllShapes();
		}

		vis.addCone(getCoefficients(0, rot, origin), "x");
		vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "x");


		vis.addCone(getCoefficients(1, rot, origin), "y");
		vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "y");

		vis.addCone(getCoefficients(2, rot, origin), "z");
		vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "z");
	}

	pcl::ModelCoefficients getCoefficients(int start, tf::Matrix3x3 rot, tf::Vector3 origin) {
		pcl::ModelCoefficients co;

		co.values.push_back(origin[0]);
		co.values.push_back(origin[1]);
		co.values.push_back(origin[2]);

		co.values.push_back(rot.getRow(start).getX());
		co.values.push_back(rot.getRow(start).getY());
		co.values.push_back(rot.getRow(start).getZ());

		co.values.push_back(1.0f);

		return co;
	}
};

MAKE_AE(TrayAnnotator)
