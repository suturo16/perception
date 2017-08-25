/**
 * @file    ColorClusterer.cpp
 * @author  Tobias Hahn
 * @date    2017-03-27
 * @version 1.0
 *
 * @brief Finds rack and clusters it by color to extract tool clusters.
 *
 * @section DESCRIPTION
 *
 * This annotator find the rack based on color, and clusters it by color to find tools.
 *
 */

/** INCLUDES **/
//UIMA
#include <uima/api.hpp>

//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/common.h>
#include <rs/utils/time.h>

//SUTURO
#include <percepteros/types/all_types.h>
#include <percepteros/HueClusterComparator.h>
#include <percepteros/ValueClusterComparator.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

//C++
#include <algorithm>
#include <limits>

/** NAMESPACES **/
using namespace uima;

/** DEFINITIONS **/
typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::Normal PointN;
typedef pcl::PointCloud<PointN> PCN;
typedef pcl::PointXYZHSV PointH;
typedef pcl::PointCloud<PointH> PCH;

class ColorClusterer : public DrawingAnnotator {
	private:
		//point clouds
		PCR::Ptr temp = PCR::Ptr(new PCR);
		PCH::Ptr cloud = PCH::Ptr(new PCH);
		PCN::Ptr normals = PCN::Ptr(new PCN);

		//indices for clusters
		std::vector<pcl::PointIndices> hue_indices;
		std::vector<pcl::PointIndices> value_indices;

		//parameters
		float DISTANCE_THRESHOLD;
		int HUE_LOWER_BOUND, HUE_UPPER_BOUND, HUE_THRESHOLD, VALUE_THRESHOLD, POINT_THRESHOLD, CLUSTER_THRESHOLD;

		/**
		 * Gets the average hue value of a cluster.
		 * @method getAverageHue
		 * @param  cloud         Point cloud containing the scene points.
		 * @param  indices       Indices of the cluster points.
		 * @return               The average hue as an int value.
		 */
		int getAverageHue(PCH::Ptr cloud, pcl::PointIndices indices) {
			float average = 0;
			int size = indices.indices.size();

			for (size_t i = 0; i < size; i++) {
				average += cloud->points[indices.indices[i]].h / size;
			}

			return (int) average;
		}

		/**
		 * Gets the average value value of a cluster.
		 * @method getAverageValue
		 * @param  cloud         Point cloud containing the scene points.
		 * @param  indices       Indices of the cluster points.
		 * @return               The average value as an int value.
		 */
		float getAverageValue(PCH::Ptr cloud, pcl::PointIndices indices) {
			float average = 0;
			int size = indices.indices.size();

			for (size_t i = 0; i < size; i++) {
				average += cloud->points[indices.indices[i]].v / size;
			}

			return average;
		}

		/**
		 * Checks if a cluster is the rack based on color.
		 * @method checkCluster
		 * @param  clust        The indices of the cluster.
		 * @param  cloud_ptr    The point cloud containing the scene points.
		 * @return              Boolean indicating if the cluster is the rack.
		 */
		bool checkCluster(rs::Cluster clust, PCH::Ptr cloud_ptr) {
			pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
			rs::ReferenceClusterPoints clusterpoints(clust.points());
			rs::conversion::from(clusterpoints.indices(), *cluster_indices);

			PointH temp;
			int count = 0;

			//Counts points of right color.
			for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
					 pit != cluster_indices->indices.end(); pit++) {
				temp = cloud_ptr->points[*pit];
				if (temp.h > HUE_LOWER_BOUND && temp.h < HUE_UPPER_BOUND) {
					count++;
				}
			}

			//Checks if there are enough points of fitting color.
			if (count > POINT_THRESHOLD) {
				return true;
			} else {
				return false;
			}
		}

		/**
		 * Annotates the rack cluster with average surface normal.
		 * @method annotateCluster
		 * @param  clust           The cluster object for the rack cluster.
		 * @param  normals         Surface normals of scene points.
		 * @param  tcas            The scene containing points and cluster information.
		 */
		void annotateCluster(rs::Cluster clust, PCN::Ptr normals, CAS &tcas) {
			//convert cluster indices to PCL indices
			pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
			rs::ReferenceClusterPoints clusterpoints(clust.points());
			rs::conversion::from(clusterpoints.indices(), *cluster_indices);

			//setup variables for extraction
			PCN::Ptr rack = PCN::Ptr(new PCN);
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

			//extract cluster normals
			pcl::ExtractIndices<PointN> ex;
			ex.setKeepOrganized(false);
			ex.setInputCloud(normals);
			ex.setIndices(cluster_indices);
			ex.filter(*rack);

			//removes invalid normal values
			std::vector<int> indices;
			removeNaNNormalsFromPointCloud(*rack, *rack, indices);

			std::vector<float> normal = getAverageNormal(rack);

			//add rack annotation
			percepteros::RackObject rA = rs::create<percepteros::RackObject>(tcas);
			rA.name.set("Rack");
			rA.normal.set(normal);
			clust.annotations.append(rA);
		}

		/**
		 * Gets the average surface normal of the point cloud provided.
		 * @method getAverageNormal
		 * @param  rack             The point cloud to calculate on.
		 * @return                  Vector with the normal components.
		 */
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

	public:
		/**
		 * Constructor method for ColorClusterer.
		 * @method BoardAnnotator
		 */
	  ColorClusterer(): DrawingAnnotator(__func__) {
	  }

		/**
		 * Initializes ColorClusterer by extracting parameters.
		 * @method initialize
		 * @param  ctx        The context object providing access to the parameters.
		 * @return            UIMA error id specifying success.
		 */
	  TyErrorId initialize(AnnotatorContext &ctx) {
	    outInfo("Initialize ColorClusterer.");

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

		/**
		 * Destructor for ColorClusterer.
		 * @method destroy
		 * @return UIMA error id specifying success.
		 */
	  TyErrorId destroy() {
	    outInfo("Destroy ColorClusterer.");
	    return UIMA_ERR_NONE;
	  }

		/**
		 * Main method, finds rack and clusters it. Called every time the pipeline is executed.
		 * @method processWithLock
		 * @param  tcas            Object containing kinect points and previous results.
		 * @param  res_spec        Specification of the expected results.
		 * @return                 UIMA error id specifying success.
		 */
	  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) {
	    outInfo("process start\n");
			//get clusters
	    	rs::SceneCas cas(tcas);
			rs::Scene scene = cas.getScene();
			std::vector<rs::Cluster> clusters;
			scene.identifiables.filter(clusters);

			//clear pointclouds
			temp->clear();
			normals->clear();
			cloud->clear();

			//get scene points
			cas.get(VIEW_CLOUD, *temp);
			cas.get(VIEW_NORMALS, *normals);
			pcl::PointCloudXYZRGBAtoXYZHSV(*temp, *cloud);
			//helpers
			rs::StopWatch clock;
			bool found = false;

			for (auto clust : clusters) {
				found = checkCluster(clust, cloud);
				if (found) {
					outInfo("Found rack!"); found = true;

					//extract cluster points
					pcl::ExtractIndices<PointH> ex;
					ex.setKeepOrganized(true);
					pcl::PointIndices::Ptr rack_i(new pcl::PointIndices);
					rs::ReferenceClusterPoints clusterpoints(clust.points());
					rs::conversion::from(clusterpoints.indices(), *rack_i);
					ex.setInputCloud(cloud);
					ex.setIndices(rack_i);
					ex.filterDirectly(cloud);

					//annotate rack cluster
					annotateCluster(clust, normals, tcas);
					pcl::PointCloud<pcl::Label>::Ptr output_labels(new pcl::PointCloud<pcl::Label>);

					//cluster rack for hue
					pcl::HueClusterComparator<PointH, pcl::Normal>::Ptr hcc(new pcl::HueClusterComparator<PointH, pcl::Normal>());
					hcc->setInputCloud(cloud);
					hcc->setDistanceThreshold(DISTANCE_THRESHOLD, true);
					hcc->setInputNormals(normals);
					hcc->setHueThreshold(HUE_THRESHOLD);

					std::vector<pcl::PointIndices> cluster_i;
					pcl::OrganizedConnectedComponentSegmentation<PointH, pcl::Label> segmenter(hcc);
					segmenter.setInputCloud(cloud);
					segmenter.segment(*output_labels, cluster_i);

					hue_indices.clear();
					for (size_t i = 0; i < cluster_i.size(); i++) {
						if (cluster_i.at(i).indices.size() > CLUSTER_THRESHOLD) {
							hue_indices.push_back(cluster_i.at(i));
						}
					}
					outInfo("Found " << hue_indices.size() << " hue clusters.");

					//remove clusters from rack
					PCH::Ptr cloud_b(new PCH());
					pcl::copyPointCloud(*cloud, *cloud_b);

					ex.setNegative(true);
					ex.setKeepOrganized(true);
					pcl::PointIndices::Ptr clust(new pcl::PointIndices());

					for (size_t i = 0; i < hue_indices.size(); i++) {
						clust->indices = hue_indices[i].indices;
						ex.setInputCloud(cloud_b);
						ex.setIndices(clust);
						ex.filterDirectly(cloud_b);
					}

					//cluster remaining rack points for value
					pcl::ValueClusterComparator<PointH, pcl::Normal>::Ptr vcc(new pcl::ValueClusterComparator<PointH, pcl::Normal>());
					vcc->setInputCloud(cloud_b);
					vcc->setDistanceThreshold(DISTANCE_THRESHOLD, true);
					vcc->setInputNormals(normals);
					vcc->setValueThreshold(VALUE_THRESHOLD);

					cluster_i.clear();
					pcl::PointCloud<pcl::Label>::Ptr output_l(new pcl::PointCloud<pcl::Label>);
					pcl::OrganizedConnectedComponentSegmentation<PointH, pcl::Label> segmenterV(vcc);
					segmenterV.setInputCloud(cloud_b);
					segmenterV.segment(*output_l, cluster_i);

					value_indices.clear();
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

		/**
		 * Visualizes results by coloring clustered points.
		 * @method fillVisualizerWithLock
		 * @param  visualizer             Visualizer oject containing points.
		 * @param  firstRun               Specifies if the visualizer is run for the first time.
		 */
		void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
			//colors hue clusters in scene
			const std::string &cloudname = this->name;
			for (size_t i = 0; i < hue_indices.size(); ++i) {
				const pcl::PointIndices &indices = hue_indices[i];
				for (size_t j = 0; j < indices.indices.size(); ++j) {
					size_t index = indices.indices[j];
					temp->points[index].rgba = rs::common::colors[i % rs::common::numberOfColors];
				}
			}

			//colors value clusters in scene
			for (size_t i = 0; i < value_indices.size(); ++i) {
				const pcl::PointIndices &indices = value_indices[i];
				for (size_t j = 0; j < indices.indices.size(); ++j) {
					size_t index = indices.indices[j];
					temp->points[index].rgba = rs::common::colors[(i + hue_indices.size()) % rs::common::numberOfColors];
				}
			}

			//adds or updates scene points to visualizer
			double pointSize = 1;
			if (firstRun) {
				visualizer.addPointCloud(temp, cloudname);
				visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
			} else {
				visualizer.updatePointCloud(temp, cloudname);
				visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
			}
		}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ColorClusterer)
