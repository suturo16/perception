#include <uima/api.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
#include <pcl/common/geometry.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types_conversion.h>

//RS
#include <rs/types/all_types.h>
#include <rs/DrawingAnnotator.h>

#include <rs/scene_cas.h>
#include <rs/utils/time.h>

//CATERROS
#include <geometry_msgs/PoseStamped.h>
#include <percepteros/types/all_types.h>
#include <percepteros/RegionDefinition.h>


using namespace uima;

typedef pcl::PointXYZRGBA PointXYZRGBA;

struct featureSet{
  Eigen::Matrix3f pca_eigen_vec;
  Eigen::Vector3f pca_eigen_vals;
  Eigen::Vector3f hsv_means;

};

class SpatulaRecognition : public DrawingAnnotator
{
private:
  //ObjectRegionFilter* orf;
  double pointSize;
  pcl::PointCloud<PointXYZRGBA>::Ptr cloud_ptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr spatula;
  std::vector<Eigen::Vector3f>  obj_position;
  std::vector<featureSet> obj_feats;
  std::vector<rs::Cluster> clusters;
  featureSet spatula_features;
  float range;
  float vector_length;
  Eigen::Vector3f spatula_pos; //this one describes the cluster center
  
  //spatula description
  bool found_spat;
  tf::Transform spat_transf;
  tf::Vector3 spat_x, spat_y, spat_z;
  pcl::PointXYZ spatula_origin; //this one describes the highest point in the spatula cluster

  featureSet computeFeatures(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
  bool hasSimilarFS(featureSet , featureSet);
  pcl::PointXYZ getOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr);
  pcl::ModelCoefficients getCoefficients(tf::Vector3 axis, pcl::PointXYZ origin);


public:

  SpatulaRecognition(): DrawingAnnotator(__func__), pointSize(1){

      cloud_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
      outInfo("initialize");
      ObjectRegionFilter &orf = ObjectRegionFilter::getInstance();
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun);
  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec); 
  TyErrorId destroy();
  TyErrorId initialize(AnnotatorContext &ctx);
};


TyErrorId SpatulaRecognition::initialize(AnnotatorContext &ctx)
{
  if(ctx.isParameterDefined("range")) ctx.extractValue("range", range);
  if(ctx.isParameterDefined("vector_length")) ctx.extractValue("vector_length", vector_length);

  if(ctx.isParameterDefined("eigen_vec_0") && ctx.isParameterDefined("eigen_vec_1") && ctx.isParameterDefined("eigen_vec_2"))
  {
    float eigen_vec_0, eigen_vec_1, eigen_vec_2;
    ctx.extractValue("eigen_vec_0", eigen_vec_0);
    ctx.extractValue("eigen_vec_1", eigen_vec_1);
    ctx.extractValue("eigen_vec_2", eigen_vec_2);
    this->spatula_features.pca_eigen_vec(0,0) = eigen_vec_0;
    this->spatula_features.pca_eigen_vec(1,0) = eigen_vec_1;
    this->spatula_features.pca_eigen_vec(2,0) = eigen_vec_2;
    outInfo("set vaule: eigen_vec_0, eigen_vec_1, eigen_vec_2");
  }

  if(ctx.isParameterDefined("eigen_val_0") && ctx.isParameterDefined("eigen_val_0") && ctx.isParameterDefined("eigen_val_0"))
  {
    float eigen_val_0, eigen_val_1, eigen_val_2;
    ctx.extractValue("eigen_val_0", eigen_val_0);
    ctx.extractValue("eigen_val_1", eigen_val_1);
    ctx.extractValue("eigen_val_2", eigen_val_2);
    this->spatula_features.pca_eigen_vals = Eigen::Vector3f(eigen_val_0, eigen_val_1, eigen_val_2);
    outInfo("set vaule: , eigen_val_1, eigen_val_2");
  }

  if(ctx.isParameterDefined("h_val") && ctx.isParameterDefined("s_val") && ctx.isParameterDefined("v_val"))
  {
    float h, s, v;
    ctx.extractValue("h_val", h);
    ctx.extractValue("s_val", s);
    ctx.extractValue("v_val", v);
    this->spatula_features.hsv_means = Eigen::Vector3f(h, s, v);
    outInfo("set vaule: h_val, s_val, v_val");
  }
 
  outInfo("initialize");
  return UIMA_ERR_NONE;
}

TyErrorId SpatulaRecognition::destroy()
{
  outInfo("destroy");
  return UIMA_ERR_NONE;
}

TyErrorId SpatulaRecognition::processWithLock(CAS &tcas, ResultSpecification const &res_spec)
{
  outInfo("process start");

  //clearing data from last run
  this->clusters.clear();
  this->obj_position.clear();
  this->obj_feats.clear();
  found_spat = false;

  /*
  rs::StopWatch clock;
  outInfo("took: " << clock.getTime() << " ms.");
  */
  //setting up scene variables
  rs::SceneCas cas(tcas);
  rs::Scene scene = cas.getScene();
  cas.get(VIEW_CLOUD,*cloud_ptr);
//  orf->getviewCloud("spatula", cloud_ptr, cloud_ptr);

  //getting "up-achis" of scene
  tf::StampedTransform camToWorld, worldToCam;
  camToWorld.setIdentity();
  if(scene.viewPoint.has())
  {
    rs::conversion::from(scene.viewPoint.get(), camToWorld);
  }
  else
  {
    outInfo("No camera to world transformation!!!");
  }
  worldToCam = tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);

  tf::Matrix3x3 matrix = worldToCam.getBasis();
  tf::Vector3 scene_z = matrix*tf::Vector3(0,0,1);

  scene.identifiables.filter(clusters);

  for (rs::Cluster cluster : clusters)
  {
    pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
    rs::ReferenceClusterPoints clusterpoints(cluster.points());
    rs::conversion::from(clusterpoints.indices(), *cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA>());
    
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

    featureSet computed_fs = computeFeatures(temp);
    this->obj_feats.push_back(computed_fs);
    Eigen::Vector4f center_temp;
    pcl::compute3DCentroid(*object_cloud, center_temp);
    Eigen::Vector3f centroid;
    centroid << center_temp.x(), center_temp.y(), center_temp.z();
    obj_position.push_back(centroid);

    if (hasSimilarFS(spatula_features, computed_fs))
    {
      outInfo("Spatula detected");
      found_spat = true;
      this->spatula_pos = centroid;
      spatula_origin = getOrigin(object_cloud);

      spat_transf.setOrigin(tf::Vector3(spatula_origin.x, spatula_origin.y, spatula_origin.z));

      spat_x = tf::Vector3(spatula_features.pca_eigen_vec(0,0), spatula_features.pca_eigen_vec(1,0), spatula_features.pca_eigen_vec(2,0));
      spat_y = scene_z.cross(spat_x);
      spat_z = spat_x.cross(spat_y);

      spat_x.normalize();
      spat_y.normalize();
      spat_z.normalize();

      
      if (
        std::isnan(spat_x[0]) || std::isnan(spat_x[1]) || std::isnan(spat_x[2]) ||
        std::isnan(spat_y[0]) || std::isnan(spat_y[1]) || std::isnan(spat_y[2]) ||
        std::isnan(spat_z[0]) || std::isnan(spat_z[1]) || std::isnan(spat_z[2])
        ) 
      {
        outError("Found wrong orientation. Abort.");
        break;
      }

      tf::Matrix3x3 spat_rot;
      spat_rot.setValue(
                    spat_x.x(), spat_y.x(), spat_z.x(), 
                    spat_x.y(), spat_y.y(), spat_z.y(), 
                    spat_x.z(), spat_y.z(), spat_z.z()
                  );
      spat_transf.setBasis(spat_rot);
      
      tf::Stamped<tf::Pose> camera(spat_transf, camToWorld.stamp_, camToWorld.child_frame_id_);
      tf::Stamped<tf::Pose> world(camToWorld*spat_transf, camToWorld.stamp_, camToWorld.frame_id_);

      rs::PoseAnnotation poseAnnotation = rs::create<rs::PoseAnnotation>(tcas);
      poseAnnotation.camera.set(rs::conversion::to(tcas, camera));
      poseAnnotation.world.set(rs::conversion::to(tcas, world));
      poseAnnotation.source.set("3DEstimate");
      cluster.annotations.append(poseAnnotation);

      //append RecognitionObject
      percepteros::RecognitionObject spatulaObj = rs::create<percepteros::RecognitionObject>(tcas);
      spatulaObj.name.set("cakeSpatula");
      spatulaObj.type.set(8);
      // HOTFIX: PLEASE REMOVE IF NOT NEEDED
      spatulaObj.width.set(0.28f);
      spatulaObj.height.set(0.056f);
      spatulaObj.depth.set(0.03f);
      cluster.annotations.append(spatulaObj);

      //put spatula into scene
      scene.identifiables.append(cluster);

    }
  }

  if (obj_feats.size() != obj_position.size())
  {
    outInfo("Number of available object positions does not match number of availabe object feature sets!");
    return UIMA_ERR_NONE;
  }

  outInfo("process stop");
  return UIMA_ERR_NONE;
}


void SpatulaRecognition::fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
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

  outInfo("total obj no " + std::to_string(obj_feats.size()));

  if (this->found_spat)
  {
    visualizer.addText3D("spatula", pcl::PointXYZ(this->spatula_pos.x(), this->spatula_pos.y(), this->spatula_pos.z()), 0.02);;
    visualizer.addCone(getCoefficients(spat_x, spatula_origin), "x");
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "x");
    visualizer.addCone(getCoefficients(spat_y, spatula_origin), "y");
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "y");
    visualizer.addCone(getCoefficients(spat_z, spatula_origin), "z");
    visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "z");
  }

  if (obj_feats.size() == obj_position.size() )
  {
    for (int i = 0; i < obj_feats.size(); i++)
    {
      std::string obj_desc("");
      obj_desc += "eV: " + std::to_string(obj_feats[i].pca_eigen_vec(0,0)) + ", " + std::to_string(obj_feats[i].pca_eigen_vec(1,0)) + ", " + std::to_string(obj_feats[i].pca_eigen_vec(2,0)) + "/n" ;
    }
  }  
}

featureSet SpatulaRecognition::computeFeatures(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster)
{
  featureSet cluster_feats;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr space_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::copyPointCloud(*cluster, *space_cloud);

  space_cloud->width = space_cloud->points.size();
  space_cloud->height = 1;
  space_cloud->is_dense = true;
  
  //get Eigenvectors
  pcl::PCA<pcl::PointXYZ> cluster_axis;
  cluster_axis.setInputCloud(space_cloud);
  cluster_feats.pca_eigen_vec = cluster_axis.getEigenVectors();

  //compute their magnitude
  cluster_feats.pca_eigen_vals = cluster_axis.getEigenValues();
  
  //compute rgb centroid
  pcl::CentroidPoint<pcl::PointXYZRGBA> centroidComputer;
  for(auto point = cluster->begin(); point != cluster->end(); point++)
    centroidComputer.add(*point);
  pcl::PointXYZRGBA centroid;
  centroidComputer.get(centroid);
  
  pcl::PointXYZHSV hsv;
  pcl::PointXYZRGBAtoXYZHSV(centroid, hsv);
  cluster_feats.hsv_means = Eigen::Vector3f(hsv.h, hsv.s, hsv.v);

  return cluster_feats;
}

bool SpatulaRecognition::hasSimilarFS(featureSet compared, featureSet comparing)
{
  //outInfo("hasSimilarFS started");
  //check eigenvectors
  Eigen::Vector3f compared_eigen_vec;
  compared_eigen_vec << compared.pca_eigen_vec(0,0), compared.pca_eigen_vec(1,0), compared.pca_eigen_vec(2,0); 
  Eigen::Vector3f comparing_eigen_vec;
  comparing_eigen_vec << comparing.pca_eigen_vec(0,0), comparing.pca_eigen_vec(1,0), comparing.pca_eigen_vec(2,0); 
  float eigen_vec_dist = (compared_eigen_vec - comparing_eigen_vec).squaredNorm();
  //outInfo("vec distance" << eigen_vec_dist);
  if (eigen_vec_dist>range) return false;

  //check EigenValues
  float eigen_val_dist = (compared.pca_eigen_vals - comparing.pca_eigen_vals).squaredNorm();
  //outInfo("val distance" << eigen_val_dist);
  if (eigen_val_dist > range) return false;

  //check hsv
  float hsv_dist = (compared.hsv_means- comparing.hsv_means).squaredNorm();
  //outInfo("hsv distance" << hsv_dist);
  if (hsv_dist > range) return false;

  //outInfo("hasSimilarFS ended");

  return true;
}

pcl::ModelCoefficients SpatulaRecognition::getCoefficients(tf::Vector3 axis, pcl::PointXYZ origin) {
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

pcl::PointXYZ SpatulaRecognition::getOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr spat) {
  pcl::PointXYZ spatula_origin,begin, end;
  std::vector<pcl::PointXYZ> endpoints(2);
  int size = spat->size(); 
  float currDistance = 0;
  
  for (int i = 0; i < size; i++) {
    begin = spat->points[i];
    for (int j = i+1; j < size; j++) {
      end = spat->points[j];
      if (pcl::geometry::distance(begin, end) > currDistance) {
        endpoints[0] = begin;
        endpoints[1] = end;
        currDistance = pcl::geometry::distance(begin, end);
      }
    }
  }

  if (endpoints[0].x + endpoints[0].y + endpoints[0].z < endpoints[1].x + endpoints[1].y + endpoints[1].z) {
    spatula_origin = endpoints[0];
  } else {
    spatula_origin = endpoints[1];
  }
  return spatula_origin;

}

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(SpatulaRecognition)