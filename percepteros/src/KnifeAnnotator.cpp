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

//surface matching
#include <pcl/io/ply_io.h>
#include <Eigen/Core>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace uima;

typedef pcl::PointNormal PointG;
typedef pcl::PointCloud<PointG> PCG;
typedef pcl::Normal PointN;
typedef pcl::PointXYZ PointT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointG,PointG,FeatureT> FET;
typedef pcl::PointCloud<FeatureT> FCT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointG> ColorHandlerT;

class KnifeAnnotator : public DrawingAnnotator
{
private:
  pcl::PointCloud<PointT>::Ptr cloud_ptr;
	pcl::PointCloud<PointN>::Ptr normal_ptr;
	PCG::Ptr cloud_normal_ptr;
	PCG::Ptr model_ptr;
	PCG::Ptr model_aligned;

	static const size_t PPF_LENGTH = 5;

public:

  KnifeAnnotator(): DrawingAnnotator(__func__){
      cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
			normal_ptr = pcl::PointCloud<PointN>::Ptr(new pcl::PointCloud<PointN>);
			cloud_normal_ptr = PCG::Ptr(new PCG);
			model_ptr = PCG::Ptr(new PCG);
			model_aligned = PCG::Ptr(new PCG);
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
    outInfo("process start\n");
    rs::SceneCas cas(tcas);
		rs::Scene scene = cas.getScene();
		pcl::PLYReader reader;
		FCT::Ptr object_f(new FCT);
		FCT::Ptr cloud_f(new FCT);
		rs::StopWatch clock;

		outInfo("getting clouds\n");
		//get points and normals
    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normal_ptr);
		pcl::concatenateFields(*cloud_ptr, *normal_ptr, *cloud_normal_ptr);

		//get points from ply file
		reader.read("../data/knife.ply", *model_ptr);
		
		//downsampling
		outInfo("downsampling\n");
		pcl::VoxelGrid<PointG> grid;
		const float leaf = 0.005f;
		grid.setLeafSize(leaf, leaf, leaf);
		grid.setInputCloud(cloud_normal_ptr);
		grid.filter(*cloud_normal_ptr);
		grid.setInputCloud(model_ptr);
		grid.filter(*model_ptr);
		
		outInfo("At " << clock.getTime() << "ms.");

		//estimating features
		outInfo("estimating features\n");
		FET fest;
		fest.setRadiusSearch(0.025);
		fest.setInputCloud(cloud_normal_ptr);
		fest.setInputNormals(cloud_normal_ptr);
		fest.compute(*cloud_f);
		fest.setInputCloud(model_ptr);
		fest.setInputNormals(model_ptr);
		fest.compute(*object_f);

		outInfo("At " << clock.getTime() << "ms.");		

		//aligning
		outInfo("start alignment\n");
		pcl::SampleConsensusPrerejective<PointG,PointG,FeatureT> align;
		align.setInputSource(cloud_normal_ptr);
		align.setSourceFeatures(cloud_f);
		align.setInputTarget(model_ptr);
		align.setTargetFeatures(object_f);
		align.setMaximumIterations(50000); //ransac iterations
		align.setNumberOfSamples(3);
		align.setCorrespondenceRandomness(5); //number of nearest features to use
		align.setSimilarityThreshold(0.9f); //polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance(2.5f * leaf); //inlier threshold
		align.setInlierFraction(0.25f); // required inlier fraction for accepting a pose hypothesis
		align.align(*model_aligned);

		outInfo("At " << clock.getTime() << "ms.");
		
		if (align.hasConverged()) {
			outInfo("finished alignment");
			Eigen::Matrix4f transformation = align.getFinalTransformation();
			/*
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
			pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
			pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
			pcl::console::print_info ("\n");
			pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
			pcl::console::print_info ("\n");
			pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), model_ptr->size ());
			
			//show alignment
			pcl::visualization::PCLVisualizer visu("Alignment");
			visu.addPointCloud (cloud_normal_ptr, ColorHandlerT (cloud_normal_ptr, 0.0, 255.0, 0.0), "scene");
			visu.addPointCloud (model_aligned, ColorHandlerT (model_aligned, 0.0, 0.0, 255.0), "model_aligned");
			visu.spin ();
			*/
			//publishing results
			geometry_msgs::PoseStamped pose;
			percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);
			tf::Transform transform;

			Eigen::Matrix3f mat;
			mat << transformation(0,0), transformation(0,1), transformation(0,2),
						 transformation(1,0), transformation(1,1), transformation(1,2),
						 transformation(2,0), transformation(2,1), transformation(2,2);

			Eigen::Quaternionf qua(mat);
			qua.normalize();

			tf::Vector3 trans(transformation(3,0), transformation(3,1), transformation(3,2));
			tf::Matrix3x3 rot;
			rot.setValue(mat(0,0), mat(0,1), mat(0,2),
									 mat(1,0), mat(1,1), mat(1,2),
									 mat(2,0), mat(2,1), mat(2,2));

			transform.setOrigin(trans);
			transform.setBasis(rot);

			pose.header.frame_id = cloud_ptr->header.frame_id;
			pose.pose.orientation.x = qua.x();
			pose.pose.orientation.y = qua.y();
			pose.pose.orientation.z = qua.z();
			pose.pose.orientation.w = qua.w();

			pose.pose.position.x = transformation(3,0);
			pose.pose.position.y = transformation(3,1);
			pose.pose.position.z = transformation(3,2);

			o.name.set("Knife");
			o.type.set(6);
			o.width.set(0);
			o.height.set(0);
			o.depth.set(0);

			tf::StampedTransform camToWorld;
			camToWorld.setIdentity();
			if (scene.viewPoint.has()) {
				rs::conversion::from(scene.viewPoint.get(), camToWorld);
			}

			tf::Stamped<tf::Pose> camera(transform, camToWorld.stamp_, camToWorld.child_frame_id_);
			tf::Stamped<tf::Pose> world(camToWorld*transform, camToWorld.stamp_, camToWorld.frame_id_);

			rs::PoseAnnotation poseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
			poseAnnotation.camera.set(rs::conversion::to(tcas, camera));
			poseAnnotation.world.set(rs::conversion::to(tcas, world));
			poseAnnotation.source.set("3DEstimate");
			
			rs::Cluster cluster;
			cluster.annotations.append(o);
			cluster.annotations.append(poseAnnotation);
			scene.identifiables.append();

		} else {
			outInfo("alignment not finished");
		}

    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(KnifeAnnotator)
