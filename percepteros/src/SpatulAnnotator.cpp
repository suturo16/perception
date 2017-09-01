//INCLUDES
//UIMA
#include <uima/api.hpp>

//RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/types/all_types.h>

//SUTURO
#include <percepteros/types/all_types.h>

//ROS
#include <geometry_msgs/PoseStamped.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/point_types_conversion.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/geometry.h>

//NAMESPACES
using namespace uima;

//DEFINITIONS
typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::PointXYZHSV PointH;
typedef pcl::PointCloud<PointH> PCH;

class SpatulAnnotator : public DrawingAnnotator
{
	private:
		PCR::Ptr cloud_r = PCR::Ptr(new PCR);
		PCH::Ptr cloud_h = PCH::Ptr(new PCH);
		PCH::Ptr spatula = PCH::Ptr(new PCH);
		float LINE_DISTANCE, MIN_INLIERS;

		pcl::ModelCoefficients getCoefficients(tf::Vector3 axis, PointH origin) {
			pcl::ModelCoefficients coeffs;
			//point
			coeffs.values.push_back(origin.x);
			coeffs.values.push_back(origin.y);
			coeffs.values.push_back(origin.z);
			//direction
			coeffs.values.push_back(axis[0]);
			coeffs.values.push_back(axis[1]);
			coeffs.values.push_back(axis[2]);
			//radius
			coeffs.values.push_back(1.0f);

			return coeffs;
		}

		float getGreatestDistance(PointH middle) {
			float greatest = 0;

			for (PointH p : spatula->points) {
				if (pcl::geometry::distance(middle, p) > greatest) {
					greatest = pcl::geometry::distance(middle, p);
				}
			}

			return greatest;
		}

		PointH getOrigin(PCH::Ptr spatula) {
		  PointH origin, begin, end;
		  std::vector<PointH> endpoints(2);
		  int size = spatula->size();
		  float currDistance = 0;

		  for (int i = 0; i < size; i++) {
		    begin = spatula->points[i];
		    for (int j = i+1; j < size; j++) {
		      end = spatula->points[j];
		      if (pcl::geometry::distance(begin, end) > currDistance) {
		        endpoints[0] = begin;
		        endpoints[1] = end;
		        currDistance = pcl::geometry::distance(begin, end);
		      }
		    }
		  }

		  if (endpoints[0].x + endpoints[0].y + endpoints[0].z < endpoints[1].x + endpoints[1].y + endpoints[1].z) {
		    origin = endpoints[0];
		  } else {
		    origin = endpoints[1];
		  }
		  return origin;
		}

	public:
		tf::Vector3 x, y, z;
		PointH origin;

		SpatulAnnotator(): DrawingAnnotator(__func__) {
		}

		TyErrorId initialize(AnnotatorContext &ctx) {
			outInfo("Initialize SpatulAnnotation.");

			//extract color parameters
			ctx.extractValue("lineDistance", LINE_DISTANCE);
			ctx.extractValue("inliers", MIN_INLIERS);

			return UIMA_ERR_NONE;
		}

		TyErrorId destroy()	{
			outInfo("Destroy SpatulAnnotation.");
			return UIMA_ERR_NONE;
		}

		TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) {
			outInfo("Start SpatulaAnnotation.");

			//get clusters
			rs::SceneCas cas(tcas);
			rs::Scene scene = cas.getScene();
			std::vector<rs::Cluster> clusters;
			scene.identifiables.filter(clusters);

			//getting "up-axis" of scene
		  tf::StampedTransform camToWorld, worldToCam;
		  camToWorld.setIdentity();
		  if(scene.viewPoint.has()) {
		    rs::conversion::from(scene.viewPoint.get(), camToWorld);
		  } else {
		    outInfo("No camera to world transformation!");
		  }
		  worldToCam = tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);
		  tf::Matrix3x3 matrix = worldToCam.getBasis();
		  z = matrix*tf::Vector3(0,0,1);

			//get scene points
			cas.get(VIEW_CLOUD, *cloud_r);
			pcl::PointCloudXYZRGBAtoXYZHSV(*cloud_r, *cloud_h);

			//prepare segmenter
			pcl::SACSegmentation<PointH> seg;
			seg.setOptimizeCoefficients(true);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setMaxIterations(1000);
			seg.setModelType(pcl::SACMODEL_LINE);
			seg.setDistanceThreshold(LINE_DISTANCE);

			//prepare line pointers
			pcl::ModelCoefficients::Ptr lco(new pcl::ModelCoefficients());
			pcl::PointIndices::Ptr lin(new pcl::PointIndices());

			//prepare extractor
			pcl::ExtractIndices<PointH> ex;
			ex.setInputCloud(cloud_h);

			//find cluster
			rs::Cluster spatula_cluster = clusters[0];
			bool foundSpatula = false;
			for (rs::Cluster cluster : clusters) {
				pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		    rs::ReferenceClusterPoints clusterpoints(cluster.points());
		    rs::conversion::from(clusterpoints.indices(), *cluster_indices);

				//extract points
				ex.setIndices(cluster_indices);
				ex.filter(*spatula);

				//segment
				seg.setInputCloud(spatula);
				seg.segment(*lin, *lco);

				//check inlier ratio
				float inlier = lin->indices.size();
				float size = spatula->points.size();
				float ratio = inlier / size;

				//check average value
				if (ratio > MIN_INLIERS) {
					spatula_cluster = cluster;
					foundSpatula = true;
					break;
				}
			}

			if (!foundSpatula) {
				outInfo("Didn't find a spatula.");
				return UIMA_ERR_NONE;
			}

			pcl::PCA<PointH> pca;
			pca.setInputCloud(spatula);
			Eigen::Matrix3f vecs = pca.getEigenVectors();
			Eigen::Vector4f orig = pca.getMean();

			//set vectors
			x.setValue(vecs(0,0), vecs(1,0), vecs(2,0));
			y = z.cross(x);
			z = x.cross(y);
			x.normalize(); y.normalize(); z.normalize();

			//set origin
			origin = getOrigin(spatula);

			return UIMA_ERR_NONE;
		}



		void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
			if (firstRun) {
				visualizer.addPointCloud(cloud_r, "scene points");
			} else {
				visualizer.updatePointCloud(cloud_r, "scene points");
				visualizer.removeAllShapes();
			}

			visualizer.addCone(getCoefficients(x, origin), "x");
			visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "x");

			visualizer.addCone(getCoefficients(y, origin), "y");
			visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "y");

			visualizer.addCone(getCoefficients(z, origin), "z");
			visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "z");
		}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SpatulAnnotator)
