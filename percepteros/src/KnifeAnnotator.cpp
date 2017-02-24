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
#include <pcl/io/obj_io.h>
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
typedef pcl::PointXYZRGBA PointW;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointG,PointG,FeatureT> FET;
typedef pcl::PointCloud<FeatureT> FCT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointG> ColorHandlerT;

class KnifeAnnotator : public DrawingAnnotator
{
private:
	pcl::PointCloud<PointW>::Ptr temp_ptr;
  pcl::PointCloud<PointT>::Ptr cloud_ptr;
	pcl::PointCloud<PointN>::Ptr normal_ptr;
	PCG::Ptr cloud_normal_ptr;
	PCG::Ptr model_ptr;
	PCG::Ptr model_aligned;

	static const size_t PPF_LENGTH = 5;

public:

  KnifeAnnotator(): DrawingAnnotator(__func__){
			temp_ptr = pcl::PointCloud<PointW>::Ptr(new pcl::PointCloud<PointW>);
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
		pcl::OBJReader reader;
		FCT::Ptr object_f(new FCT);
		FCT::Ptr cloud_f(new FCT);
		rs::StopWatch clock;

		outInfo("getting clouds\n");
		//get points and normals
    cas.get(VIEW_CLOUD, *temp_ptr);
		pcl::copyPointCloud(*temp_ptr, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normal_ptr);
		pcl::concatenateFields(*cloud_ptr, *normal_ptr, *cloud_normal_ptr);

		//get points from ply file
		reader.read("/home/tobias/cateros/src/perception/percepteros/data/KNIFE_FINAL.obj", *model_ptr);

		//downsampling
		outInfo("downsampling\n");
		pcl::VoxelGrid<PointG> grid;
		const float leaf = 0.1f;
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
		outInfo("Compute cloud");
		fest.compute(*cloud_f);
		fest.setInputCloud(model_ptr);
		fest.setInputNormals(model_ptr);
		outInfo("compute object");
		fest.compute(*object_f);

		outInfo("At " << clock.getTime() << "ms.");		

		//aligning
		outInfo("start alignment\n");
		pcl::SampleConsensusPrerejective<PointG,PointG,FeatureT> align;
		align.setInputSource(cloud_normal_ptr);
		align.setSourceFeatures(cloud_f);
		align.setInputTarget(model_ptr);
		align.setTargetFeatures(object_f);
		align.setMaximumIterations(500000); //ransac iterations
		align.setNumberOfSamples(3);
		align.setCorrespondenceRandomness(5); //number of nearest features to use
		align.setSimilarityThreshold(0.5f); //polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance(2.0f * leaf); //inlier threshold
		align.setInlierFraction(0.1f); // required inlier fraction for accepting a pose hypothesis
		align.align(*model_aligned);

		outInfo("At " << clock.getTime() << "ms.");
		
		if (align.hasConverged()) {
			outInfo("finished alignment");
			Eigen::Matrix4f transformation = align.getFinalTransformation();
			
			//publishing results
			percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);
			tf::Transform transform;
			
			tf::Vector3 trans(transformation(0,3), transformation(1,3), transformation(2,3));
			tf::Matrix3x3 rot;
			rot.setValue(transformation(0,0), transformation(0,1), transformation(0,2),
									 transformation(1,0), transformation(1,1), transformation(1,2),
									 transformation(2,0), transformation(2,1), transformation(2,2));
			transform.setOrigin(trans);
			transform.setBasis(rot);
			
			outInfo("Transform: " << transformation(0,3) << transformation(1,3) << transformation(2,3));

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
			
			rs::Cluster cluster = rs::create<rs::Cluster>(tcas);
			cluster.annotations.append(o);
			cluster.annotations.append(poseAnnotation);
			scene.identifiables.append(cluster);
		} else {
			outInfo("alignment not finished");
		}

    return UIMA_ERR_NONE;
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(KnifeAnnotator)
