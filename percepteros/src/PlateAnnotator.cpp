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

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/extract_indices.h>

#include <cmath>
#include <sstream>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::Normal	Normal;
typedef pcl::PointCloud<Normal> PCN;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN> PC;

class PlateAnnotator : public DrawingAnnotator
{
private:
	PCR::Ptr cloud_r = PCR::Ptr(new PCR);
	PCN::Ptr cloud_n = PCN::Ptr(new PCN);
	PC::Ptr cloud = PC::Ptr(new PC);
	PC::Ptr clust = PC::Ptr(new PC);
	PC::Ptr clust_filtered = PC::Ptr(new PC);

	std::vector<pcl::ModelCoefficients> plates;
	std::vector<std::vector<tf::Vector3>> poses;

public:
	const float MAX_DIST_CENTS = 0.1;
	const float MAX_RATIO_RADII = 0.8;

  PlateAnnotator(): DrawingAnnotator(__func__){
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
		
		//extract color parameters
		//ctx.extractValue("minHue", HUE_LOWER_BOUND);
		//ctx.extractValue("maxHue", HUE_UPPER_BOUND);

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
	for (auto it = clusters.begin(); it != clusters.end(); ++it) {
		auto cluster = *it;
		shapes.clear();
		cluster.annotations.filter(shapes);
		outInfo(cluster.source.get());
		outInfo(cluster.source.get().compare(0, 13, "HueClustering"));
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
	
				ex.setInputCloud(clust);
				ex.setIndices(cin1);
				ex.filter(*clust_filtered);

				//second circle
				seg.setInputCloud(clust_filtered);
				seg.segment(*cin2, *cco2);
				
				//printCoefficients("Second circle", cco2);
				

				if 	(isPlate(cco1, cco2, (int) std::strtof(cluster.source.get().erase(0, 15).data(), NULL))) {
					outInfo("found plate");
					plates.push_back(*cco1);
					addAnnotation(tcas, cluster, *cco1, clust->points[cin1->indices[0]]);
				}
			}
		}
		return UIMA_ERR_NONE;
	}
  

	void addAnnotation(CAS &tcas, rs::Cluster cluster, pcl::ModelCoefficients co, PointN circ) {
		//calculate values
		std::vector<tf::Vector3> po;
		tf::Vector3 x,y,z,origin;
		origin.setValue(co.values[0], co.values[1], co.values[2]);
		x.setValue(circ.x - co.values[0], circ.y - co.values[1], circ.z - co.values[2]);
		z.setValue(co.values[4], co.values[5], co.values[6]);
		y = x.cross(z);
		x.normalize(); y.normalize(); z.normalize();
		
		po.push_back(x); po.push_back(y); po.push_back(z); po.push_back(origin);
		poses.push_back(po);
	
		tf::Matrix3x3 rot;
		rot.setValue(	x[0], x[1], x[2],
						y[0], y[1], y[2],
						z[0], z[1], z[2]);
	
	tf::Transform trans;
		trans.setOrigin(origin);
		trans.setBasis(rot);
			
		//create annotation
		rs::PoseAnnotation poseA = rs::create<rs::PoseAnnotation>(tcas);
		tf::StampedTransform camToWorld;
		camToWorld.setIdentity();
		
		tf::Stamped<tf::Pose> camera(trans, camToWorld.stamp_, camToWorld.child_frame_id_);
		tf::Stamped<tf::Pose> world(camToWorld * trans, camToWorld.stamp_, camToWorld.frame_id_);
		
		poseA.source.set("PlateAnnotator");
		poseA.camera.set(rs::conversion::to(tcas, camera));
		poseA.world.set(rs::conversion::to(tcas, world));
	
		//adjust recognition object
		percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);
	
		o.name.set("Plate");
		o.type.set(7);
		o.color.set(std::stoi(cluster.source.get().substr(15)));
		o.width.set(co.values[3]);
		o.height.set(0);
		o.depth.set(0);
	
		cluster.annotations.append(o);
		cluster.annotations.append(poseA);
	}

	bool isPlate(pcl::ModelCoefficients::Ptr co1, pcl::ModelCoefficients::Ptr co2, int avHue) {
		return avHue > 200 && avHue < 350 && (sqrt(pow(co1->values[0] - co2->values[0], 2) + pow(co1->values[1] - co2->values[1], 2) + pow(co1->values[2] - co2->values[2], 2)) < MAX_DIST_CENTS);
	}

	void extractCluster(PC::Ptr clust, PC::Ptr cloud, rs::Cluster cluster) {
		clust->clear();
		pcl::PointIndices::Ptr clix(new pcl::PointIndices);
		rs::ReferenceClusterPoints clups(cluster.points());
		rs::conversion::from(clups.indices(), *clix);

		for (std::vector<int>::const_iterator pit = clix->indices.begin(); pit != clix->indices.end(); pit++) {
			clust->push_back(cloud->points[*pit]);
		}
	}

	void printCoefficients(std::string name, pcl::ModelCoefficients::Ptr co) {
		outInfo(name);
		outInfo("x=" << co->values[0] << " y=" << co->values[1] << " z=" << co->values[2] << " R=" << co->values[3]);
	}

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
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(PlateAnnotator)
