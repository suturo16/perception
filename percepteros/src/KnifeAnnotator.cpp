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
	FCT::Ptr model_f;

public:

  KnifeAnnotator(): DrawingAnnotator(__func__){
			temp_ptr = pcl::PointCloud<PointW>::Ptr(new pcl::PointCloud<PointW>);
      cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
			normal_ptr = pcl::PointCloud<PointN>::Ptr(new pcl::PointCloud<PointN>);
			cloud_normal_ptr = PCG::Ptr(new PCG);
			model_ptr = PCG::Ptr(new PCG);
			model_f = FCT::Ptr(new FCT);
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
		//get clusters
    rs::SceneCas cas(tcas);
		rs::Scene scene = cas.getScene();
		std::vector<rs::Cluster> clusters;
		scene.identifiables.filter(clusters);
		
		outInfo("Number of clusters: " << clusters.size());

		//get scene clusters
		cas.get(VIEW_CLOUD, *temp_ptr);
		cas.get(VIEW_NORMALS, *normal_ptr);
		pcl::copyPointCloud(*temp_ptr, *cloud_ptr);
		pcl::concatenateFields(*cloud_ptr, *normal_ptr, *cloud_normal_ptr);

		//get points from obj file
		pcl::OBJReader reader;
		reader.read("/home/tobias/cateros/src/perception/percepteros/data/BEST_SIDE_M.obj", *model_ptr);

		//downsampling
		pcl::VoxelGrid<PointG> grid;
		const float leaf = 0.005f;
		grid.setLeafSize(leaf, leaf, leaf);
		grid.setInputCloud(model_ptr);
		grid.filter(*model_ptr);

		//estimating features
		FET fest;
		fest.setRadiusSearch(0.025);
		fest.setInputCloud(model_ptr);
		fest.setInputNormals(model_ptr);
		fest.compute(*model_f);
		
		//aligning
		pcl::SampleConsensusPrerejective<PointG,PointG,FeatureT> align;
		align.setMaximumIterations(50000); //ransac iterations
		align.setNumberOfSamples(10);
		align.setCorrespondenceRandomness(5); //number of nearest features to use
		align.setSimilarityThreshold(0.9f); //polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance(2.5f * leaf); //inlier threshold
		align.setInlierFraction(0.25f); // required inlier fraction for accepting a pose hypothesis
		
		bool finished = false;
		rs::StopWatch clock;

		for (auto cluster : clusters) {
			Eigen::Matrix4f transformation;
			PCG::Ptr cluster_ptr = extractPoints(cluster, cloud_normal_ptr, grid);
			FCT::Ptr cluster_f = extractFeatures(cluster_ptr, fest);
			doTransformation(model_aligned, model_ptr, model_f, cluster_ptr, cluster_f, align);
			
			if (align.hasConverged()) {
				transformation = align.getFinalTransformation();
				finished = true;
				outInfo("finished alignment");
				
				//can't give tcas to function, so have to do it here
				percepteros::RecognitionObject o = rs::create<percepteros::RecognitionObject>(tcas);
				tf::Transform transform = publishResults(cluster, transformation, o);
				
				rs::PoseAnnotation poseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
				tf::StampedTransform camToWorld;
				camToWorld.setIdentity();
				if (scene.viewPoint.has()) {
					rs::conversion::from(scene.viewPoint.get(), camToWorld);
				}

				tf::Stamped<tf::Pose> camera(transform, camToWorld.stamp_, camToWorld.child_frame_id_);
				tf::Stamped<tf::Pose> world(camToWorld * transform, camToWorld.stamp_, camToWorld.frame_id_);

				poseAnnotation.camera.set(rs::conversion::to(tcas, camera));
				poseAnnotation.world.set(rs::conversion::to(tcas, world));
				poseAnnotation.source.set("Estimate");

				cluster.annotations.append(o);
				cluster.annotations.append(poseAnnotation);
				scene.identifiables.append(cluster);
				break;
			}
		}
		
		if (!finished) {
			outInfo("No matching cluster found");
		}
		
		outInfo("Finished recognition in: " << clock.getTime() << "ms.");

    return UIMA_ERR_NONE;
  }

	FCT::Ptr extractFeatures(PCG::Ptr input, FET fest) {
		//initialize feature object
		FCT::Ptr input_f(new FCT);

		//compute features
		fest.setInputCloud(input);
		fest.setInputNormals(input);
		fest.compute(*input_f);
		
		//return feature object
		return input_f;
	}

	PCG::Ptr extractPoints(rs::Cluster cluster, PCG::Ptr cloud_ptr, pcl::VoxelGrid<PointG> grid) {
		PCG::Ptr cluster_ptr(new PCG);
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
				 pit != cluster_indices->indices.end(); pit++) {
			cluster_ptr->push_back(cloud_ptr->points[*pit]);		 
		}

		cluster_ptr->width = cluster_ptr->points.size();
		cluster_ptr->height = 1;
		cluster_ptr->is_dense = true;

		grid.setInputCloud(cluster_ptr);
		grid.filter(*cluster_ptr);

		return cluster_ptr;
	}

	void doTransformation(PCG::Ptr model_aligned, PCG::Ptr model_ptr, FCT::Ptr model_f, PCG::Ptr cluster_ptr, FCT::Ptr cluster_f, pcl::SampleConsensusPrerejective<PointG,PointG,FeatureT> align) {
		align.setInputSource(model_ptr);
		align.setSourceFeatures(model_f);
		align.setInputTarget(cluster_ptr);
		align.setTargetFeatures(cluster_f);
		align.align(*model_aligned);
	}

	tf::Transform publishResults(rs::Cluster cluster, Eigen::Matrix4f transformation, percepteros::RecognitionObject o) {
		tf::Transform transform;
		
		tf::Vector3 trans(transformation(0,3), transformation(1,3), transformation(2,3));
		tf::Matrix3x3 rot;
		rot.setValue(transformation(0,0), transformation(0,1), transformation(0,2),
								 transformation(1,0), transformation(1,1), transformation(1,2),
								 transformation(2,0), transformation(2,1), transformation(2,2));

		transform.setOrigin(trans);
		transform.setBasis(rot);

		o.name.set("Knife");
		o.type.set(6);
		o.width.set(0);
		o.height.set(0);
		o.depth.set(0);
		
		return transform;
	}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(KnifeAnnotator)
