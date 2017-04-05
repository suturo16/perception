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

// OTHER
#include <percepteros/types/all_types.h>
#include <rs/segmentation/ImageSegmentation.h>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::Normal N;
typedef pcl::PointCloud<N> PN;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN> PCN;

class TrayAnnotator : public DrawingAnnotator
{

private:
	PCR::Ptr temp = PCR::Ptr(new PCR);
	PN::Ptr tempN = PN::Ptr(new PN);
	PCN::Ptr cloud = PCN::Ptr(new PCN);
	PCN::Ptr tray = PCN::Ptr(new PCN);

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

    cas.get(VIEW_CLOUD, *temp);
		cas.get(VIEW_NORMALS, *tempN);
		pcl::copyPointCloud(*temp, *cloud);
		pcl::copyPointCloud(*tempN, *cloud);

		std::vector<rs::Cluster> clusters;
		scene.identifiables.filter(clusters);
		
		pcl::ModelCoefficients coefficients;
		pcl::PointIndices inliers;

		pcl::SACSegmentationFromNormals<PointN, PointN> seg;
		seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(2000);
		seg.setDistanceThreshold(0.1);
		seg.setAxis(Eigen::Vector3f(1,0,0));
		seg.setEpsAngle(0.01);

		for (auto cluster : clusters) {
			if (cluster.source.get().compare(0, 13, "HueClustering") == 0) {
				pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
				rs::ReferenceClusterPoints clusterpoints(cluster.points());
				rs::conversion::from(clusterpoints.indices(), *cluster_indices);

				extractPoints(cloud, tray, cluster_indices);

				seg.setInputCloud(tray);
				seg.setInputNormals(tray);
				seg.segment(inliers, coefficients);
				
				outInfo(inliers.indices.size() / tray->points.size());
				if (inliers.indices.size() / tray->points.size() > 0.9) {
					outInfo(cluster.source.get());
				}
			}
		}
		return UIMA_ERR_NONE;
  }

	void extractPoints(PCN::Ptr cloud, PCN::Ptr object, pcl::PointIndices::Ptr indices) {
		object->clear();
		for (size_t i = 0; i < indices->indices.size(); i++) {
			object->push_back(cloud->points[i]);
		}
	}
};

MAKE_AE(TrayAnnotator)
