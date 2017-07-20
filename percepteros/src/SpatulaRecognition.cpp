#include <uima/api.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>

//RS
#include <rs/types/all_types.h>
#include <rs/DrawingAnnotator.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>



using namespace uima;

typedef pcl::PointXYZRGBA PointT;


class SpatulaRecognition : public DrawingAnnotator
{
private:
  double pointSize;
  pcl::PointCloud<PointT>::Ptr cloud_ptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr spatula;
  std::vector<Eigen::Matrix3f> obj_orientation;
  std::vector<Eigen::Vector3f>  obj_position;
  float radius;
  float vector_length;
public:

  SpatulaRecognition(): DrawingAnnotator(__func__), pointSize(1){

      cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    if(ctx.isParameterDefined("radius")) ctx.extractValue("radius", radius);
    if(ctx.isParameterDefined("vector_length")) ctx.extractValue("vector_length", vector_length);
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
    //rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();
    cas.get(VIEW_CLOUD,*cloud_ptr);

    outInfo("Cloud size: " << cloud_ptr->points.size());
//    outInfo("took: " << clock.getTime() << " ms.");

    std::vector<rs::Cluster> clusters;
    scene.identifiables.filter(clusters);

    for (rs::Cluster cluster : clusters)
    {
      pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
      rs::ReferenceClusterPoints clusterpoints(cluster.points());
      rs::conversion::from(clusterpoints.indices(), *cluster_indices);

      pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
      
      for(std::vector<int>::const_iterator pit = cluster_indices->indices.begin();
          pit != cluster_indices->indices.end(); pit++)
      {
        temp->points.push_back(cloud_ptr->points[*pit]);
      }
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::copyPointCloud(*temp, *object_cloud);

      object_cloud->width = object_cloud->points.size();
      object_cloud->height = 1;
      object_cloud->is_dense = true;

      pcl::PCA<pcl::PointXYZ> spat_axis;
      spat_axis.setInputCloud(object_cloud);
      obj_orientation.push_back(spat_axis.getEigenVectors());
      outInfo("obj mat : " << obj_orientation.back());

      Eigen::Vector4f center_temp;
      pcl::compute3DCentroid(*object_cloud, center_temp);
      Eigen::Vector3f centroid;
      centroid << center_temp.x(), center_temp.y(), center_temp.z();
      outInfo("obj position" << centroid);
      obj_position.push_back(centroid);
      /*
      */
    }

    if (obj_orientation.size() != obj_position.size())
    {
      outInfo("Number of available object positions does not match number of availabe object orientations!");
      return UIMA_ERR_NONE;
    }

    return UIMA_ERR_NONE;
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {
    
    std::string cloudname("scene points");
    
    if (firstRun){
      outInfo("1Cloud size: " << cloud_ptr->points.size());
      visualizer.addPointCloud(cloud_ptr, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    } else {
      outInfo("0Cloud size: " << cloud_ptr->points.size());
      visualizer.updatePointCloud(cloud_ptr, cloudname);
      visualizer.removeAllShapes();
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
    }

    //double radius = 0.5;
    if (obj_orientation.size() == obj_position.size())
    {
      outInfo("total obj no " + std::to_string(obj_orientation.size()));
      for (int i = 0; i<obj_position.size(); i++)
      {
        pcl::PointXYZ pos(obj_position[i].x(), obj_position[i].y(), obj_position[i].z());
        visualizer.addSphere(pos, radius,1, 0, 0, std::to_string(i));
        pcl::PointXYZ to_1(vector_length*(obj_position[i].x()-obj_orientation[i](0,0)), vector_length*(obj_position[i].y()-obj_orientation[i](0,1)), vector_length*(obj_position[i].z()-obj_orientation[i](0,2)));
        //pcl::PointXYZ to_2(vector_length*(obj_position[i].x()-obj_orientation[i](0, 0)), vector_length*(obj_position[i].y()-obj_orientation[i](1, 0)), vector_length*(obj_position[i].z()-obj_orientation[i](2, 0)));
        visualizer.addLine(pos, to_1, 1, 0, 0, std::to_string(i)+"_a");
        visualizer.addLine(pos, to_1, 0, 1, 0, std::to_string(i)+"_b");
        /*

        pcl::ModelCoefficients mc;
        mc.values.push_back(obj_position[i].x());
        mc.values.push_back(obj_position[i].y());
        mc.values.push_back(obj_position[i].z());
        mc.values.push_back(obj_orientation[0][0]);
        mc.values.push_back(obj_orientation.y());
        mc.values.push_back(obj_orientation.z());
        visualizer.addCone( mc ,"x");
        visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "x");
*/
      }
    }
  }
};



// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SpatulaRecognition)