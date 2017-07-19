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
#include <pcl/common/transforms.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <tf_conversions/tf_eigen.h>

#include <cmath>
#include <sstream>
#include <typeinfo>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;

class BoardAnnotator : public DrawingAnnotator
{
private:
	PCR::Ptr cloud = PCR::Ptr(new PCR);
	PCR::Ptr neighbors = PCR::Ptr(new PCR);
	PCR::Ptr board = PCR::Ptr(new PCR);
	pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);

	double width, height, depth;
	tf::Vector3 max, min, x, y, z, mid;
	pcl::PointXYZ middle;

public:

  BoardAnnotator(): DrawingAnnotator(__func__){
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
	
	//empty containers
	cloud->clear();
	neighbors->clear();
	board->clear();

	//get clusters
    rs::SceneCas cas(tcas);
	rs::Scene scene = cas.getScene();
	std::vector<rs::Cluster> clusters;
	scene.identifiables.filter(clusters);

	//get scene points
	cas.get(VIEW_CLOUD, *cloud);
	cas.get(VIEW_CLOUD, *neighbors);
	
	outInfo("Checking " << clusters.size() << " clusters.");
	std::vector<percepteros::RecognitionObject> recs;
	std::vector<rs::PoseAnnotation> poses;
	
	Eigen::Affine3f iTrans;
	//check for box and get dimensions
	int clusterIdx = 0;
	bool foundBox = false;
	for (int i = 0; i < clusters.size(); i++) {
		//check for box
		clusters[i].annotations.filter(recs);
		clusters[i].annotations.filter(poses);
		if (recs.size() > 0 && recs[0].type.get() == 1) {
				outInfo("Found box");
				clusterIdx = i;
				foundBox = true;
				//get dimensions
				width = recs[0].width.get();
				height = recs[0].height.get();
				depth = recs[0].depth.get();
				
				Eigen::Affine3d eTrans;
				tf::Transform tTrans;
				rs::conversion::from(poses[0].world.get(), tTrans);
				tf::transformTFToEigen(tTrans, eTrans);	
				iTrans = eTrans.inverse().cast<float>();
				pcl::transformPointCloud(*neighbors, *neighbors, eTrans);

				middle.x = poses[0].camera.get().translation.get()[0];
				middle.y = poses[0].camera.get().translation.get()[1];
				middle.z = poses[0].camera.get().translation.get()[2];
				
				middle = pcl::transformPoint(middle, eTrans.cast<float>());

				min.setValue(middle.x - 0.15, middle.y - 0.15, middle.z - 0.1);
				max.setValue(middle.x + 0.15, middle.y + 0.15, middle.z + 0.1);

				auto rot = poses[0].camera.get().rotation.get();
				
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

void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
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
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(BoardAnnotator)
