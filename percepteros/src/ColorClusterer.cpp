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
#include <percepteros/ValueClusterComparator.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <algorithm>
#include <limits>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointN> PCN;
typedef pcl::PointXYZHSV PointH;
typedef pcl::PointCloud<PointH> PCH;
/*
typedef pcl::PointNormal PointXN;
typedef pcl::PointCloud<PointXN> PCXN;
typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PCX;
*/
class ColorClusterer : public DrawingAnnotator
{
private:
	PCR::Ptr temp = PCR::Ptr(new PCR);
	PCH::Ptr cloud = PCH::Ptr(new PCH);
	PCN::Ptr normals = PCN::Ptr(new PCN);
/*	PCXN::Ptr cloud_n = PCXN::Ptr(new PCXN);
	PCX::Ptr cloud_x = PCX::Ptr(new PCX);
*/
	std::vector<pcl::PointIndices> hue_indices;
	std::vector<pcl::PointIndices> value_indices;

	float DISTANCE_THRESHOLD;
	int HUE_LOWER_BOUND, HUE_UPPER_BOUND, HUE_THRESHOLD, VALUE_THRESHOLD, POINT_THRESHOLD, CLUSTER_THRESHOLD;

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
		ctx.extractValue("diffVal", VALUE_THRESHOLD);

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
/*		
		pcl::copyPointCloud(*temp, *cloud_x);
		pcl::concatenateFields(*cloud_x, *normals, *cloud_n);
*/		//helpers
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
				annotateCluster(clust, normals, tcas);				
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
						hue_indices.push_back(cluster_i.at(i));
					}
				}
				outInfo("Found " << hue_indices.size() << " hue clusters.");
				
				//remove clusters from rack
				PCH::Ptr cloud_b(new PCH());
				pcl::copyPointCloud(*cloud, *cloud_b);

				pcl::ExtractIndices<PointH> ex;
				ex.setNegative(true);
				ex.setKeepOrganized(true);
				pcl::PointIndices::Ptr clust(new pcl::PointIndices());

				for (size_t i = 0; i < hue_indices.size(); i++) {		
					clust->indices = hue_indices[i].indices;
					ex.setInputCloud(cloud_b);
					ex.setIndices(clust);
					ex.filterDirectly(cloud_b);
				}
				
				pcl::ValueClusterComparator<PointH, pcl::Normal, pcl::Label>::Ptr vcc(new pcl::ValueClusterComparator<PointH, pcl::Normal, pcl::Label>());
				vcc->setInputCloud(cloud_b);
				vcc->setLabels(input_labels);
				vcc->setExcludeLabels(ignore_labels);
				vcc->setDistanceThreshold(DISTANCE_THRESHOLD, true);
				vcc->setInputNormals(normals);
				vcc->setValueThreshold(VALUE_THRESHOLD);
				
				cluster_i.clear();
				pcl::PointCloud<pcl::Label>::Ptr output_l(new pcl::PointCloud<pcl::Label>);
				pcl::OrganizedConnectedComponentSegmentation<PointH, pcl::Label> segmenterV(vcc);
				segmenterV.setInputCloud(cloud_b);
				segmenterV.segment(*output_l, cluster_i);
					
				for (size_t i = 0; i < cluster_i.size(); i++) {
					if (cluster_i.at(i).indices.size() > CLUSTER_THRESHOLD) {
						value_indices.push_back(cluster_i.at(i));
					}
				}
				outInfo("Found " << value_indices.size() << " value clusters.");
				
				//append clusters to scene
				//TODO: Add rois for 2d
				for	(size_t i = 0; i < hue_indices.size(); ++i) {
					const pcl::PointIndices &indices = hue_indices[i];

					rs::Cluster uimaCluster = rs::create<rs::Cluster>(tcas);
					rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
					rs::PointIndices uimaIndices = rs::conversion::to(tcas, indices);

					rcp.indices.set(uimaIndices);

					uimaCluster.points.set(rcp);
					uimaCluster.source.set("HueClustering");

					percepteros::ToolObject tool = rs::create<percepteros::ToolObject>(tcas);
					tool.name.set("HueClustering");
					int averageHue = getAverageHue(cloud, indices);
					float averageValue = getAverageValue(cloud, indices);
					tool.hue.set(averageHue);
					tool.value.set(averageValue);
					uimaCluster.annotations.append(tool);

					scene.identifiables.append(uimaCluster);
				}

				for	(size_t i = 0; i < value_indices.size(); ++i) {
					const pcl::PointIndices &indices = value_indices[i];

					rs::Cluster uimaCluster = rs::create<rs::Cluster>(tcas);
					rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
					rs::PointIndices uimaIndices = rs::conversion::to(tcas, indices);

					rcp.indices.set(uimaIndices);

					uimaCluster.points.set(rcp);
					uimaCluster.source.set("ValueClustering");

					percepteros::ToolObject tool = rs::create<percepteros::ToolObject>(tcas);
					tool.name.set("ValueClustering");
					int averageHue = getAverageHue(cloud, indices);
					float averageValue = getAverageValue(cloud, indices);
					tool.hue.set(averageHue);
					tool.value.set(averageValue);
					uimaCluster.annotations.append(tool);

					scene.identifiables.append(uimaCluster);
				}
				break;
			}
		}
		
		if (!found) {
			outInfo("No matching cluster");
		}

		outInfo("Finished clustering in: " << clock.getTime() << "ms.");
    return UIMA_ERR_NONE;
  }

	int getAverageHue(PCH::Ptr cloud, pcl::PointIndices indices) {
		float average = 0;
		int size = indices.indices.size();

		for (size_t i = 0; i < size; i++) {
			average += cloud->points[indices.indices[i]].h / size;
		}

		return (int) average;
	}

	float getAverageValue(PCH::Ptr cloud, pcl::PointIndices indices) {
		float average = 0;
		int size = indices.indices.size();

		for (size_t i = 0; i < size; i++) {
			average += cloud->points[indices.indices[i]].v / size;
		}

		return average;
	}

	void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
		const std::string &cloudname = this->name;
		for (size_t i = 0; i < hue_indices.size(); ++i) {
			const pcl::PointIndices &indices = hue_indices[i];
			for (size_t j = 0; j < indices.indices.size(); ++j) {
				size_t index = indices.indices[j];
				temp->points[index].rgba = rs::common::colors[i % rs::common::numberOfColors];
			}
		}

		for (size_t i = 0; i < value_indices.size(); ++i) {
			const pcl::PointIndices &indices = value_indices[i];
			for (size_t j = 0; j < indices.indices.size(); ++j) {
				size_t index = indices.indices[j];
				temp->points[index].rgba = rs::common::colors[(i + hue_indices.size()) % rs::common::numberOfColors];
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

	void annotateCluster(rs::Cluster clust, PCN::Ptr normals, CAS &tcas) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(clust.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);
		
		PCN::Ptr rack = PCN::Ptr(new PCN);
		pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		pcl::ExtractIndices<PointN> ex;
		ex.setKeepOrganized(false);
		ex.setInputCloud(normals);
		ex.setIndices(cluster_indices);
		ex.filter(*rack);
		
		std::vector<int> indices;
		removeNaNNormalsFromPointCloud(*rack, *rack, indices);
/*
        //prepare segmenter
		pcl::SACSegmentationFromNormals<PointXN, PointXN> seg;
		seg.setOptimizeCoefficients(true);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(500);
		seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
		seg.setDistanceThreshold(10);
		seg.setProbability(0.1);
		seg.setInputCloud(rack);
		seg.setInputNormals(rack);

		seg.segment(*inliers, *coeffs);*/
		std::vector<float> normal = getAverageNormal(rack);
		//add rack annotation
		percepteros::RackObject rA = rs::create<percepteros::RackObject>(tcas);
		rA.name.set("Rack");
		//std::vector<float> normal; normal.push_back(coeffs->values[0]); normal.push_back(coeffs->values[1]); normal.push_back(coeffs->values[2]);
		rA.normal.set(normal);
		clust.annotations.append(rA);
	}

	std::vector<float> getAverageNormal(PCN::Ptr rack) {
		std::vector<float> avn(3);
		int size = rack->points.size();

		for (int i = 0; i < size; i++) {
			avn[0] -= rack->points[i].normal_x / size;
			avn[1] -= rack->points[i].normal_y / size;
			avn[2] -= rack->points[i].normal_z / size;
		}
		
		return avn;
	}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ColorClusterer)
