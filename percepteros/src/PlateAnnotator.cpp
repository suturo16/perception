/**
 * @file    PlateAnnotator.cpp
 * @author  Tobias Hahn
 * @date    2017-05-05
 * @version 1.0
 *
 * @brief Finds pose of plate in scene.
 *
 * @section DESCRIPTION
 *
 * This annotator find the plate in the scene, based on geometric information.
 *
 */

/** INCLUDES **/
//UIMA
#include <uima/api.hpp>

//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/time.h>

//SUTURO
#include <percepteros/types/all_types.h>

//ROS
#include <geometry_msgs/PoseStamped.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/extract_indices.h>

//C++
#include <cmath>
#include <sstream>

/** NAMESPACES **/
using namespace uima;

/** DEFINITIONS **/
typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::Normal	Normal;
typedef pcl::PointCloud<Normal> PCN;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN> PC;

class PlateAnnotator : public DrawingAnnotator {
	private:
		//clouds
		PCR::Ptr cloud_r = PCR::Ptr(new PCR);
		PCN::Ptr cloud_n = PCN::Ptr(new PCN);
		PC::Ptr cloud = PC::Ptr(new PC);
		PC::Ptr clust = PC::Ptr(new PC);
		PC::Ptr clust_filtered = PC::Ptr(new PC);

		//poses of plates
		std::vector<std::vector<tf::Vector3>> poses;

		//parameters
		int HUE_LOWER_BOUND, HUE_UPPER_BOUND;

		/**
		 * Adds an annotation to the plate cluster.
		 * @method addAnnotation
		 * @param  tcas          Object containing points and cluster information.
		 * @param  cluster       Object containing cluster indices.
		 * @param  co            Model coefficients for circle representing plate.
		 * @param  circ          Middle point of circle.
		 */
		void addAnnotation(CAS &tcas, rs::Cluster cluster, pcl::ModelCoefficients co, PointN circ) {
			//calculate average normal
			extractCluster(clust, cloud, cluster);
			std::vector<int> indices;
			removeNaNNormalsFromPointCloud(*clust, *clust, indices);

			tf::Vector3 normal = getAverageNormal(clust);

			//calculate pose values
			std::vector<tf::Vector3> po;
			tf::Vector3 x,y,z,origin;
			origin.setValue(co.values[0], co.values[1], co.values[2]);
			x.setValue(circ.x - co.values[0], circ.y - co.values[1], circ.z - co.values[2]);
			z.setValue(co.values[4], co.values[5], co.values[6]);
			if (z.angle(normal) > 1.8f) {
				z.setValue(-co.values[4], -co.values[5], -co.values[6]);
			}

			y = x.cross(z);
			x = y.cross(z);
			z = x.cross(y);

			x.normalize(); y.normalize(); z.normalize();

			//saving pose for visualization
			po.push_back(x); po.push_back(y); po.push_back(z); po.push_back(origin);
			poses.push_back(po);

			tf::Matrix3x3 rot;

			rot.setValue(	x[0], y[0], z[0],
							x[1], y[1], z[1],
							x[2], y[2], z[2]);

			tf::Transform trans;
			trans.setOrigin(origin);
			trans.setBasis(rot);

			//create annotation
			rs::PoseAnnotation poseA = rs::create<rs::PoseAnnotation>(tcas);
			tf::StampedTransform camToWorld;
			camToWorld.setIdentity();

			rs::SceneCas cas(tcas);
			rs::Scene scene = cas.getScene();
			if (scene.viewPoint.has()) {
				rs::conversion::from(scene.viewPoint.get(), camToWorld);
			}

			tf::Stamped<tf::Pose> camera(trans, camToWorld.stamp_, camToWorld.child_frame_id_);
			tf::Stamped<tf::Pose> world(camToWorld * trans, camToWorld.stamp_, camToWorld.frame_id_);

			poseA.source.set("PlateAnnotator");
			poseA.camera.set(rs::conversion::to(tcas, camera));
			poseA.world.set(rs::conversion::to(tcas, world));

			//adjust recognition object
			percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);

			o.name.set("dinnerPlateForCake");
			o.type.set(7);
			o.width.set(co.values[3]);
			o.height.set(0);
			o.depth.set(0);

			cluster.annotations.append(poseA);
			cluster.annotations.append(o);
		}

		/**
		 * Calculates average surface normal for cluster.
		 * @method getAverageNormal
		 * @param  plate            Point cloud containing cluster points.
		 * @return                  Average surface normal components in a tf::Vector3.
		 */
		tf::Vector3 getAverageNormal(PC::Ptr plate) {
			std::vector<float> avn(3);
			int size = plate->points.size();

			for (int i = 0; i < size; i++) {
				avn[0] += plate->points[i].normal_x / size;
				avn[1] += plate->points[i].normal_y / size;
				avn[2] += plate->points[i].normal_z / size;
			}

			tf::Vector3 n(avn[0], avn[1], avn[2]);
			return n;
		}

		/**
		 * Detects if a cluster is a plate based on coefficients of circles found.
		 * @method isPlate
		 * @param  co1     Coefficients of first circle found.
		 * @param  co2     Coefficients of second circle found.
		 * @param  avHue   Average hue of cluster points.
		 * @return         Boolean indicating if cluster is a plate.
		 */
		bool isPlate(pcl::ModelCoefficients::Ptr co1, pcl::ModelCoefficients::Ptr co2, int avHue) {
			float dist = sqrt(pow(co1->values[0] - co2->values[0], 2) + pow(co1->values[1] - co2->values[1], 2) + pow(co1->values[2] - co2->values[2], 2));
			return (dist < MAX_DIST_CENTS);
		}

		/**
		 * Extracts points belonging to cluster into a seperate point cloud.
		 * @method extractCluster
		 * @param  clust          Point cloud to extract into.
		 * @param  cloud          Point cloud containing scene points.
		 * @param  cluster        Object containing cluster indices.
		 */
		void extractCluster(PC::Ptr clust, PC::Ptr cloud, rs::Cluster cluster) {
			clust->clear();
			pcl::PointIndices::Ptr clix(new pcl::PointIndices);
			rs::ReferenceClusterPoints clups(cluster.points());
			rs::conversion::from(clups.indices(), *clix);

			for (std::vector<int>::const_iterator pit = clix->indices.begin(); pit != clix->indices.end(); pit++) {
				clust->push_back(cloud->points[*pit]);
			}
		}

		/**
		 * Prints coefficients of circle to console.
		 * @method printCoefficients
		 * @param  name              Name of cluster.
		 * @param  co                Model coefficients of detected circle model.
		 */
		void printCoefficients(std::string name, pcl::ModelCoefficients::Ptr co) {
			outInfo(name);
			outInfo("x=" << co->values[0] << " y=" << co->values[1] << " z=" << co->values[2] << " R=" << co->values[3]);
		}

		/**
		 * Adds the pose of one plate to the visualizer.
		 * @method addPose
		 * @param  pose       Vector containing pose information.
		 * @param  visualizer Visualizer object to add visualiziations to.
		 * @param  index      Index of plate for keeping names individual.
		 */
		void addPose(std::vector<tf::Vector3> pose, pcl::visualization::PCLVisualizer &visualizer, int index) {
			std::ostringstream ss;
			ss << "x" << index;
			visualizer.addCone(getCoefficients(pose[0], pose[3]), ss.str());
			visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, ss.str());

			ss << "y" << index;
			visualizer.addCone(getCoefficients(pose[1], pose[3]), ss.str());
			visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, ss.str());

			ss << "z" << index;
			visualizer.addCone(getCoefficients(pose[2], pose[3]), ss.str());
			visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, ss.str());
		}

		/**
		 * Get model coefficients for a cone visualizing one axis.
		 * @method getCoefficients
		 * @param  axis            The axis to visualize.
		 * @param  origin          The point of origin of the object, used as starting point of the cone.
		 * @return                 ModelCoefficients for cone.
		 */
		pcl::ModelCoefficients getCoefficients(tf::Vector3 axis, tf::Vector3 origin) {
			pcl::ModelCoefficients coeffs;

			//origin
			coeffs.values.push_back(origin[0]);
			coeffs.values.push_back(origin[1]);
			coeffs.values.push_back(origin[2]);
			//axis
			coeffs.values.push_back(axis[0]);
			coeffs.values.push_back(axis[1]);
			coeffs.values.push_back(axis[2]);
			//radius
			coeffs.values.push_back(0.5f);

			return coeffs;
		}

	public:
		//constants
		const float MAX_DIST_CENTS = 0.1;
		const float MAX_RATIO_RADII = 0.8;

		/**
		 * Constructor for PlateAnnotator
		 * @method PlateAnnotator
		 */
	  PlateAnnotator(): DrawingAnnotator(__func__) {
	  }

		/**
		 * Initializes PlateAnnotator by extracting parameters.
		 * @method initialize
		 * @param  ctx        Context object providing access to parameters.
		 * @return            UIMA error id specifying success.
		 */
	  TyErrorId initialize(AnnotatorContext &ctx) {
	    outInfo("Initialize PlateAnnotator.");

			//extract color parameters
			ctx.extractValue("minHue", HUE_LOWER_BOUND);
			ctx.extractValue("maxHue", HUE_UPPER_BOUND);

	    return UIMA_ERR_NONE;
	  }

		/**
		 * Destructor for Plate Annotator.
		 * @method destroy
		 * @return UIMA error id specifying success.
		 */
	  TyErrorId destroy() {
	    outInfo("Destroying PlateAnnotator.");
	    return UIMA_ERR_NONE;
	  }

		/**
		 * Main method, detects plates and annotates them. Called every time the pipeline is executed.
		 * @method processWithLock
		 * @param  tcas            Object containing scene points and previous results.
		 * @param  res_spec        Specification of expected results.
		 * @return                 UIMA error id specifying success.
		 */
	  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) {
	    outInfo("Starting plate detection.");
			rs::StopWatch clock;

			//get clusters
	    rs::SceneCas cas(tcas);
			rs::Scene scene = cas.getScene();
			std::vector<rs::Cluster> clusters;
			clusters.clear();
			scene.identifiables.filter(clusters);

			//get scene points
			cas.get(VIEW_CLOUD, *cloud_r);
			cas.get(VIEW_NORMALS, *cloud_n);
			pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*cloud_r, *temp);
			pcl::concatenateFields(*temp, *cloud_n, *cloud);

			//prepare segmenter
			pcl::SACSegmentation<PointN> seg;
			seg.setOptimizeCoefficients(true);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setMaxIterations(500);
			seg.setModelType(pcl::SACMODEL_CIRCLE3D);
			seg.setDistanceThreshold(0.005);
			seg.setRadiusLimits(0.025, 0.13);

			//prepare extractor
			pcl::ExtractIndices<PointN> ex;
			ex.setNegative(true);

			std::vector<rs::Shape> shapes;
			poses.clear();
			for (auto it = clusters.begin(); it != clusters.end(); ++it) {
				auto cluster = *it;
				shapes.clear();
				cluster.annotations.filter(shapes);
				if (cluster.source.get().compare(0, 13, "HueClustering") > -1 &&
						shapes.size() > 0 &&
						shapes[0].shape.get().compare("round") > -1) {
						outInfo("Found a cluster");
						//could be a plate - check for two circles
						//extract cluster
						extractCluster(clust, cloud, cluster);
						//variables
						pcl::PointIndices::Ptr cin1(new pcl::PointIndices());
						pcl::PointIndices::Ptr cin2(new pcl::PointIndices());

						pcl::ModelCoefficients::Ptr cco1(new pcl::ModelCoefficients());
						pcl::ModelCoefficients::Ptr cco2(new pcl::ModelCoefficients());

						//first circle
						seg.setInputCloud(clust);
						seg.segment(*cin1, *cco1);

						//printCoefficients("First circle", cco1);

						//remove points of first circle
						ex.setInputCloud(clust);
						ex.setIndices(cin1);
						ex.filter(*clust_filtered);

						//second circle
						seg.setInputCloud(clust_filtered);
						seg.segment(*cin2, *cco2);

						//printCoefficients("Second circle", cco2);


						if 	(isPlate(cco1, cco2, (int) std::strtof(cluster.source.get().erase(0, 15).data(), NULL))) {
							outInfo("Found a plate in " << clock.getTime() << "ms.");
							addAnnotation(tcas, cluster, *cco1, clust->points[cin1->indices[0]]);
						}
					}
				}
				return UIMA_ERR_NONE;
			}

			/**
			 * Visualizes results by adding cones visualizing axes of pose.
			 * @method fillVisualizerWithLock
			 * @param  visualizer             Object to be filled with visualizing information.
			 * @param  firstRun               Indicates if visualizer is run for the first time.
			 */
			void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
				if (firstRun) {
					visualizer.addPointCloud(cloud_r, "scene");
				} else {
					visualizer.updatePointCloud(cloud_r, "scene");
					visualizer.removeAllShapes();
				}

				int i = 0;
				for (auto pose : poses) {
					addPose(pose, visualizer, i++);
				}
			}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PlateAnnotator)
