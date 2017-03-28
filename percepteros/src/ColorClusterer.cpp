#include <uima/api.hpp>

#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/common.h>
#include <rs/utils/time.h>

#include <percepteros/types/all_types.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <percepteros/HueClusterComparator.h>

#include <algorithm>
#include <limits>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointN> PCN;
typedef pcl::PointXYZHSV PointH;
typedef pcl::PointCloud<PointH> PCH;

class ColorClusterer : public DrawingAnnotator
{
private:
	PCR::Ptr temp = PCR::Ptr(new PCR);
	PCH::Ptr cloud = PCH::Ptr(new PCH);
	PCN::Ptr normals = PCN::Ptr(new PCN);

	std::vector<pcl::PointIndices> cluster_indices;

	float DISTANCE_THRESHOLD;
	int HUE_LOWER_BOUND, HUE_UPPER_BOUND, HUE_THRESHOLD, POINT_THRESHOLD, CLUSTER_THRESHOLD;

public:
  ColorClusterer(): DrawingAnnotator(__func__){
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
		
		//extract color parameters
		ctx.extractValue("minHue", HUE_LOWER_BOUND);
		ctx.extractValue("maxHue", HUE_UPPER_BOUND);
		ctx.extractValue("diffHue", HUE_THRESHOLD);

		//extract threshold
		ctx.extractValue("diffDist", DISTANCE_THRESHOLD);
		ctx.extractValue("minPoints", POINT_THRESHOLD);
		ctx.extractValue("minCluster", CLUSTER_THRESHOLD);

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
		cas.get(VIEW_CLOUD, *temp);
		cas.get(VIEW_NORMALS, *normals);
		pcl::PointCloudXYZRGBAtoXYZHSV(*temp, *cloud);

		//helpers
		rs::StopWatch clock;
		bool found = false;
		
		std::vector<bool> ignore_labels;
		pcl::PointCloud<pcl::Label>::Ptr input_labels(new pcl::PointCloud<pcl::Label>);

		ignore_labels.resize(2);
		ignore_labels[0] = true;
		ignore_labels[1] = false;

		input_labels->height = cloud->height;
		input_labels->width = cloud->width;
		input_labels->points.resize(cloud->points.size());

		for (auto clust : clusters) {
			found = checkCluster(clust, cloud, input_labels);
			if (found) {
				outInfo("Found rack!"); found = true;
				
				pcl::PointCloud<pcl::Label>::Ptr output_labels(new pcl::PointCloud<pcl::Label>);

				pcl::HueClusterComparator<PointH, pcl::Normal, pcl::Label>::Ptr hcc(new pcl::HueClusterComparator<PointH, pcl::Normal, pcl::Label>());
				hcc->setInputCloud(cloud);
				hcc->setLabels(input_labels);
				hcc->setExcludeLabels(ignore_labels);
				hcc->setDistanceThreshold(DISTANCE_THRESHOLD, true);
				hcc->setInputNormals(normals);
				hcc->setHueThreshold(HUE_THRESHOLD);

				std::vector<pcl::PointIndices> cluster_i;
				pcl::OrganizedConnectedComponentSegmentation<PointH, pcl::Label> segmenter(hcc);
				segmenter.setInputCloud(cloud);
				segmenter.segment(*output_labels, cluster_i);
				
				for (size_t i = 0; i < cluster_i.size(); i++) {
					if (cluster_i.at(i).indices.size() > CLUSTER_THRESHOLD) {
						cluster_indices.push_back(cluster_i.at(i));
					}
				}
				outInfo("Found " << cluster_indices.size() << " clusters.");
				
				//append clusters to scene
				//TODO: Add rois for 2d
				for	(size_t i = 0; i < cluster_indices.size(); ++i) {
					const pcl::PointIndices &indices = cluster_indices[i];

					rs::Cluster uimaCluster = rs::create<rs::Cluster>(tcas);
					rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
					rs::PointIndices uimaIndices = rs::conversion::to(tcas, indices);

					rcp.indices.set(uimaIndices);

					uimaCluster.points.set(rcp);
					uimaCluster.source.set("HueClustering");
					scene.identifiables.append(uimaCluster);
				}

				break;
			}
			outInfo("Finished clustering in: " << clock.getTime() << "ms.");
		}
		
		if (!found) {
			outInfo("No matching cluster");
		}

    return UIMA_ERR_NONE;
  }

	void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
		const std::string &cloudname = this->name;
		for (size_t i = 0; i < cluster_indices.size(); ++i) {
			const pcl::PointIndices &indices = cluster_indices[i];
			for (size_t j = 0; j < indices.indices.size(); ++j) {
				size_t index = indices.indices[j];
				temp->points[index].rgba = rs::common::colors[i % rs::common::numberOfColors];
			}
		}
		
		double pointSize = 1;

		if (firstRun) {
			visualizer.addPointCloud(temp, cloudname);	
			visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
		} else {
			visualizer.updatePointCloud(temp, cloudname);
			visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
		}
	}

	bool checkCluster(rs::Cluster clust, PCH::Ptr cloud_ptr, pcl::PointCloud<pcl::Label>::Ptr labels) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(clust.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);
		PointH temp;
		int count = 0;
		
		for (int i = 0; i < labels->size(); i++) {
			labels->points[i].label = 0;
		}

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
				 pit != cluster_indices->indices.end(); pit++) {
			temp = cloud_ptr->points[*pit];
			if (temp.h > HUE_LOWER_BOUND && temp.h < HUE_UPPER_BOUND) {
				count++;
			}
			labels->points[*pit].label = 1;
		}
				
		if (count > POINT_THRESHOLD) {
			return true;
		} else {
			return false;
		}
	}


};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ColorClusterer)
