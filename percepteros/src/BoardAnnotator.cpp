/**
 * @file    BoardAnnotator.cpp
 * @author  Tobias Hahn
 * @date    2017-07-06
 * @version 1.0
 *
 * @brief Calculates pose of the board holding the cake.
 *
 * @section DESCRIPTION
 *
 * This annotator calculates the pose of the board holding the cake. It does
 * so based on the pose of the cake provided by CakeAnnotator.
 *
 * The scene points are filtered based on the position and orientation of the
 * cake, and in the points next to the cake the biggest circle is detected.
 * This circle is assumed to represent the outlines of the board.
 *
 */

/** INCLUDES **/
//UIMA
#include <uima/api.hpp>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

//RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/common.h>
#include <rs/utils/time.h>
#include <rs/types/all_types.h>

//SUTURO
#include <percepteros/types/all_types.h>

//ROS
#include <geometry_msgs/PoseStamped.h>

//TF
#include <tf_conversions/tf_eigen.h>

//C++
#include <cmath>
#include <sstream>
#include <typeinfo>

/** NAMESPACES **/
using namespace uima;

/** DEFINITIONS **/
typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;

class BoardAnnotator : public DrawingAnnotator {
	private:
		//point clouds
		PCR::Ptr cloud = PCR::Ptr(new PCR);
		PCR::Ptr neighbors = PCR::Ptr(new PCR);
		PCR::Ptr board = PCR::Ptr(new PCR);

		//pointers for pcl classes
		pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

		//pose variables
		double width, height, depth;
		tf::Vector3 max, min, x, y, z, mid;
		pcl::PointXYZ middle;

		/**
		 * Gets coefficients of cone used for visualizing the axis.
		 * @method getCoeffs
		 * @param  px        x-Coordinate of origin.
		 * @param  py        y-Coordinate of origin.
		 * @param  pz        z-Coordinate of origin.
		 * @param  ax        x-Element of axis.
		 * @param  ay        y-Element of axis.
		 * @param  az        z-Element of axis.
		 * @return           ModelCoefficients for cone.
		 */
		pcl::ModelCoefficients getCoeffs(float px, float py, float pz, float ax, float ay, float az) {
			pcl::ModelCoefficients coeffs;
			//point
			coeffs.values.push_back(px);
			coeffs.values.push_back(py);
			coeffs.values.push_back(pz);
			//direction
			coeffs.values.push_back(ax);
			coeffs.values.push_back(ay);
			coeffs.values.push_back(az);
			//radius
			coeffs.values.push_back(1.0f);
			return coeffs;
		}

	public:
		/**
		 * Constructor method for BoardAnnotator.
		 * @method BoardAnnotator
		 */
	  BoardAnnotator(): DrawingAnnotator(__func__) {
	  }

		/**
		 * Initializer for BoardAnnotator, currently does nothing.
		 * @method initialize
		 * @param  ctx        The Annotator context (acces to definition of parameters).
		 * @return            UIMA error id specifying success.
		 */
	  TyErrorId initialize(AnnotatorContext &ctx) {
	    outInfo("Initialize BoardAnnotator.");

	    return UIMA_ERR_NONE;
	  }

		/**
		 * Destructor for BoardAnnotator, currently does nothing.
		 * @method destroy
		 * @return UIMA error id specifying success.
		 */
	  TyErrorId destroy() {
	    outInfo("Destroy BoardAnnotator.");
	    return UIMA_ERR_NONE;
	  }

		/**
		 * Main method, calculates pose of board based on cake pose. Called every time the pipeline is run.
		 * @method processWithLock
		 * @param  tcas            Contains kinect data and results of previous annotators.
		 * @param  res_spec        Specifications of expected results.
		 * @return                 UIMA error id specifying success.
		 */
	  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) {
	    outInfo("Start calculation of board pose.");

			//empty containers
			cloud->clear();
			neighbors->clear();
			board->clear();

			//get clusters
			rs::SceneCas cas(tcas);
			rs::Scene scene = cas.getScene();
			std::vector<rs::Cluster> clusters;
			std::vector<std::vector<percepteros::RecognitionObject>> cluster_annotions;
			scene.identifiables.filter(clusters, cluster_annotations);

			//get scene points
			cas.get(VIEW_CLOUD, *cloud);
			cas.get(VIEW_CLOUD, *neighbors);

			std::vector<rs::PoseAnnotation> poses;

			Eigen::Affine3f iTrans;
			//check for box and get dimensions
			int clusterIdx = 0;
			bool foundBox = false;
			for (int i = 0; i < clusters.size(); i++) {
				//check for box
				if (cluster_annotations[i][0].type.get() == 1) {
						outInfo("Found box.");
						clusterIdx = i;
						foundBox = true;

						//get dimensions
						perepteros::RecognitionObject rec = cluster_annotations[i][0];
						width = rec.width.get();
						height = rec.height.get();
						depth = rec.depth.get();

						//get pose
						cluster[i].annotations.filter(poses);
						rs::PoseAnnotation pose = poses[0];

						//get transformation from camera to object coordinates
						Eigen::Affine3d eTrans;
						tf::Transform tTrans;
						rs::conversion::from(pose.world.get(), tTrans);
						tf::transformTFToEigen(tTrans, eTrans);
						iTrans = eTrans.inverse().cast<float>();
						//transform point cloud to object coordinates
						pcl::transformPointCloud(*neighbors, *neighbors, eTrans);

						//gets middle point of cake
						middle.x = pose.camera.get().translation.get()[0];
						middle.y = pose.camera.get().translation.get()[1];
						middle.z = pose.camera.get().translation.get()[2];

						//transforms to object coordinates
						middle = pcl::transformPoint(middle, eTrans.cast<float>());

						//sets limits for point cloud
						min.setValue(middle.x - 0.15, middle.y - 0.15, middle.z - 0.1);
						max.setValue(middle.x + 0.15, middle.y + 0.15, middle.z + 0.1);

						auto rot = pose.camera.get().rotation.get();

						z.setValue(rot[2], rot[5], rot[8]);
						x.setValue(rot[0], rot[3], rot[6]);

						break;
				}
			}

			if (!foundBox) {
				outInfo("No box found!");
				return UIMA_ERR_NONE;
			}

			//get neighboring points
			pcl::PassThrough<PointR> pass;
			//pass.setKeepOrganized(true);

			//filter z
			pass.setInputCloud(neighbors);
			pass.setFilterFieldName("z");
			pass.setFilterLimits(min.z(), max.z());
			pass.filter(*neighbors);
			outInfo(neighbors->points.size());

			//filter y
			pass.setInputCloud(neighbors);
			pass.setFilterFieldName("y");
			pass.setFilterLimits(min.y(), max.y());
			pass.filter(*neighbors);
			outInfo(neighbors->points.size());

			//filter x
			pass.setInputCloud(neighbors);
			pass.setFilterFieldName("x");
			pass.setFilterLimits(min.x(), max.x());
			pass.filter(*neighbors);
			outInfo(neighbors->points.size());

			pcl::transformPointCloud(*neighbors, *neighbors, iTrans);

			//prepare segmenter
			pcl::SACSegmentation<PointR> seg;
			seg.setOptimizeCoefficients(true);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setMaxIterations(10000);
			seg.setModelType(pcl::SACMODEL_CIRCLE3D);
			seg.setDistanceThreshold(0.005);
			seg.setRadiusLimits(0.1, 0.15);
			seg.setInputCloud(neighbors);

			seg.segment(*indices, *coefficients);

			//create cluster
			rs::Cluster cluster = rs::create<rs::Cluster>(tcas);
			//rs::ReferenceClusterPoints rcp = rs::create<rs::ReferenceClusterPoints>(tcas);
			//rs::PointIndices indi = rs::conversion::to(tcas, *indices);

			//rcp.indices.set(indi);
			//cluster.points.set(rcp);
			cluster.source.set("BoardAnnotator");

			//calculate pose
			mid.setValue(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
			tf::Vector3 normal(coefficients->values[4], coefficients->values[5], coefficients->values[6]);
			if (z.angle(normal) > 1.8f) {
				z.setValue(-normal.getX(), -normal.getY(), -normal.getZ());
			} else {
				z.setValue(normal.getX(), normal.getY(), normal.getZ());
			}

			y = x.cross(z);
			x = y.cross(z);
			z = x.cross(y);

			x.normalize(); y.normalize(); z.normalize();

			tf::Matrix3x3 rot;
			rot.setValue(	x[0], y[0], z[0],
							x[1], y[1], z[1],
							x[2], y[2], z[2]);

			tf::Transform trans;
			trans.setOrigin(mid);
			trans.setBasis(rot);

			//create annotation
			rs::PoseAnnotation poseA = rs::create<rs::PoseAnnotation>(tcas);
			tf::StampedTransform camToWorld;
			camToWorld.setIdentity();

			if (scene.viewPoint.has()) {
				rs::conversion::from(scene.viewPoint.get(), camToWorld);
			}

			tf::Stamped<tf::Pose> camera(trans, camToWorld.stamp_, camToWorld.child_frame_id_);
			tf::Stamped<tf::Pose> world(camToWorld * trans, camToWorld.stamp_, camToWorld.frame_id_);

			poseA.source.set("BoardAnnotator");
			poseA.camera.set(rs::conversion::to(tcas, camera));
			poseA.world.set(rs::conversion::to(tcas, world));

			//create recognition object
			percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);

			o.name.set("board");
			o.type.set(9);
			o.width.set(coefficients->values[3]);
			o.height.set(0);
			o.depth.set(0);

			cluster.annotations.append(poseA);
			cluster.annotations.append(o);
			scene.identifiables.append(cluster);

			return UIMA_ERR_NONE;
	  }

		/**
		 * Adds scene points and cones illustrating the pose of the board.
		 * @method fillVisualizerWithLock
		 * @param  visualizer             PCL Visualizer class visualizing results.
		 * @param  firstRun               Specifies if Visualizer is run for the first time.
		 */
		void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
			////circle
		  //for (size_t i = 0; i < indices->indices.size(); ++i) {
		  //	cloud->points[indices->indices[i]].rgba = rs::common::colors[0];
		  //}

		  if (firstRun) {
			  visualizer.addPointCloud(cloud, "scene points");
		  } else {
			  visualizer.updatePointCloud(cloud, "scene points");
			  visualizer.removeAllShapes();
			  //visualizer.removePointCloud("neighbors");
		  }

		  //axes
		  visualizer.addCone(getCoeffs(mid.getX(), mid.getY(), mid.getZ(), x.getX(), x.getY(), x.getZ()), "x");
		  visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "x");
		  visualizer.addCone(getCoeffs(mid.getX(), mid.getY(), mid.getZ(), y.getX(), y.getY(), y.getZ()), "y");
		  visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "y");
		  visualizer.addCone(getCoeffs(mid.getX(), mid.getY(), mid.getZ(), z.getX(), z.getY(), z.getZ()), "z");
		  visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "z");
	  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BoardAnnotator)
