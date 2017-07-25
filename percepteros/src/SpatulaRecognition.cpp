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


using namespace uima;

typedef pcl::PointXYZRGBA PointXYZRGBA;

struct featureSet{
  Eigen::Matrix3f pca_eigen_vec;
  Eigen::Vector3f pca_eigen_vals;
  float h_mean; 
  float s_mean; 
  float v_mean; 
  //float max_dist; //btw two points within the cloud  
};

class SpatulaRecognition : public DrawingAnnotator
{
private:
  double pointSize;
  pcl::PointCloud<PointXYZRGBA>::Ptr cloud_ptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr spatula;
  //std::vector<Eigen::Matrix3f> obj_orientation;
  std::vector<Eigen::Vector3f>  obj_position;
  std::vector<featureSet> obj_feats;
  std::vector<rs::Cluster> clusters;
  featureSet spatula_features;
  float range;
  float vector_length;
  Eigen::Vector3f spatula_pos; //this one is for the pcl::visualizer::addtext3d
  bool found_spat;
  //pcl::PointXYZ spatula_origin; //this one is for the frame
  tf::Transform spat_transf;

  float maxDist(pcl::PointCloud<pcl::PointXYZ>::Ptr);
  featureSet computeFeatures(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr);
  bool hasSimilarFS(featureSet , featureSet);
  pcl::PointXYZ getOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr);

public:

  SpatulaRecognition(): DrawingAnnotator(__func__), pointSize(1){

      cloud_ptr = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
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
  float eigen_vec;
  if(ctx.isParameterDefined("eigen_vec_0"))
  {
    ctx.extractValue("eigen_vec_0", eigen_vec);
    this->spatula_features.pca_eigen_vec(0, 0) = eigen_vec;
    outInfo("set vaule: eigen_vec_0");
  }
  if(ctx.isParameterDefined("eigen_vec_1"))
  {
    ctx.extractValue("eigen_vec_1", eigen_vec);
    this->spatula_features.pca_eigen_vec(1, 0) = eigen_vec;  
    outInfo("set vaule: eigen_vec_1");
  }
  if(ctx.isParameterDefined("eigen_vec_2"))
  {
    ctx.extractValue("eigen_vec_2", eigen_vec);
    this->spatula_features.pca_eigen_vec(1, 2) = eigen_vec;
    outInfo("set vaule: eigen_vec_2");
  }

  float eigen_val;
  if(ctx.isParameterDefined("eigen_val_0"))
  {
    ctx.extractValue("eigen_val_0", eigen_val);
    this->spatula_features.pca_eigen_vals[0] = eigen_val;
    outInfo("set vaule: eigen_val_0");
  }
  if(ctx.isParameterDefined("eigen_val_1"))
  {
    ctx.extractValue("eigen_val_1", eigen_val);
    this->spatula_features.pca_eigen_vals[1] = eigen_val;
    outInfo("set vaule: eigen_vec_1");
  }
  if(ctx.isParameterDefined("eigen_val_2"))
  {
    ctx.extractValue("eigen_val_2", eigen_val);
    this->spatula_features.pca_eigen_vals[2] = eigen_val;
    outInfo("set vaule: eigen_val_2");
  }
  float hsv;
  if(ctx.isParameterDefined("h_val"))
  {
    ctx.extractValue("h_val", hsv);
    this->spatula_features.h_mean = hsv;
    outInfo("set vaule: h_val");
  }
  if(ctx.isParameterDefined("s_val"))
  {
    ctx.extractValue("s_val", hsv);
    this->spatula_features.s_mean = hsv;
    outInfo("set vaule: s_val");
  }
  if(ctx.isParameterDefined("v_val"))
  {
    ctx.extractValue("v_val", hsv);
    this->spatula_features.v_mean = hsv;
    outInfo("set vaule: v_val");
  }
/*
  if(ctx.isParameterDefined("max_dist")) 
  {
      ctx.extractValue("max_dist", this->spatula_features.max_dist);
      outInfo("set value: max_dist");
  }
*/  
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
  //this->obj_orientation.clear();

  /*
  rs::StopWatch clock;
  outInfo("took: " << clock.getTime() << " ms.");
  */
  //setting up scene variables
  rs::SceneCas cas(tcas);
  rs::Scene scene = cas.getScene();
  cas.get(VIEW_CLOUD,*cloud_ptr);

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
      pcl::PointXYZ spatula_origin = getOrigin(object_cloud);

/*    Vorschlag: Ys = Xs x Z; Zs = Xs x Ys
      Wobei 's' die Achsen des Pfannenwenders sind und Z die globale Z-Achse
      und 'x' das Kreuzprodukt
*/
      spat_transf.setOrigin(tf::Vector3(spatula_origin.x, spatula_origin.y, spatula_origin.z));

      tf::Vector3 spat_x(spatula_features.pca_eigen_vec(0,0), spatula_features.pca_eigen_vec(1,0), spatula_features.pca_eigen_vec(2,0));
      tf::Vector3 spat_y = spat_x.cross(scene_z);
      tf::Vector3 spat_z = spat_x.cross(spat_y);

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
      scene.identifiables.append(cluster);
      /*
      */
    }
    /*
    */
  }

  if (obj_feats.size() != obj_position.size())
  {
    outInfo("Number of available object positions does not match number of availabe object feature sets!");
    return UIMA_ERR_NONE;
  }

  outInfo("process stop");
  return UIMA_ERR_NONE;
}

/*
struct featureSet{
  Eigen::Matrix3f pca_eigen_vec;
  Eigen::Vector3f pca_eigen_vals;
  double h_mean; 
  double s_mean; 
  double v_mean; 
};
*/

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
  }
  /*
  if (obj_feats.size() == obj_position.size())
  {
    for (int i = 0; i<obj_position.size(); i++)
    {
      pcl::PointXYZ pos(obj_position[i].x(), obj_position[i].y(), obj_position[i].z());
      pcl::PointXYZ to(vector_length*(obj_position[i].x()-obj_feats[i].pca_eigen_vec(0, 0)), vector_length*(obj_position[i].y()-obj_feats[i].pca_eigen_vec(1, 0)), vector_length*(obj_position[i].z()-obj_feats[i].pca_eigen_vec(2, 0)));
      visualizer.addLine(pos, to, 1, 0, 0, std::to_string(i)+"_a");
      std::string obj_description;
      obj_description = std::to_string(obj_feats[i].pca_eigen_vec(0,0))+", "+std::to_string(obj_feats[i].pca_eigen_vec(1,0))+", "+std::to_string(obj_feats[i].pca_eigen_vec(2,0))+"\n"
                        +std::to_string(obj_feats[i].pca_eigen_vals[0])+", "+std::to_string(obj_feats[i].pca_eigen_vals[1])+", "+std::to_string(obj_feats[i].pca_eigen_vals[2])+"\n"
                        +std::to_string(obj_feats[i].h_mean)+", "+std::to_string(obj_feats[i].s_mean)+", "+std::to_string(obj_feats[i].v_mean);
      //visualizer.addText3D(obj_description.c_str(), pos, 0.005);
      if(hasSimilarFS(spatula_features, this->obj_feats[i]))
      {
        outInfo("detected Spatula!!!");
        visualizer.addText3D("spatula"+std::to_string(i), pos, 0.02);
      }
    }
  }
  else
  {
    outError("the sizes of obj_feats and obj_position do not match!");
  }
  */
}

float SpatulaRecognition::maxDist(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  pcl::PointXYZ tmp_begin, tmp_end, begin, end;
  std::vector<pcl::PointXYZ> endpoints(2);
  int size = cluster->size(); 
  float currDistance = 0;    
  for (int i = 0; i < size; i++) {
    begin = cluster->points[i];
    for (int j = i+1; j < size; j++) {
      end = cluster->points[j];
      if (pcl::geometry::distance(begin, end) > currDistance) {
        endpoints[0] = begin;
        endpoints[1] = end;
        currDistance = pcl::geometry::distance(begin, end);
      }
    }
  }
  return currDistance;
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
  cluster_feats.h_mean = hsv.h;
  cluster_feats.s_mean = hsv.s;
  cluster_feats.v_mean = hsv.v;
  
  //get max distance within cloud
  //cluster_feats.max_dist = std::abs(maxDist(space_cloud));

  return cluster_feats;
}

bool SpatulaRecognition::hasSimilarFS(featureSet compared, featureSet comparing)
{
  //float range = 0.1;
  //check eigenvectors
  //check eigenvalues
  Eigen::Vector3f eV_range = compared.pca_eigen_vals * 0.1;
  Eigen::Vector3f eV_min = compared.pca_eigen_vals - eV_range;
  Eigen::Vector3f eV_max = compared.pca_eigen_vals + eV_range;

  if((comparing.pca_eigen_vals[0] < eV_min[0])||(comparing.pca_eigen_vals[1] < eV_min[1])||(comparing.pca_eigen_vals[2] < eV_min[2])) return false;
  if((comparing.pca_eigen_vals[0] > eV_max[0])||(comparing.pca_eigen_vals[1] > eV_max[1])||(comparing.pca_eigen_vals[2] > eV_max[2])) return false;

  //check hsv
  if (((compared.h_mean-(range*compared.h_mean))>comparing.h_mean)||((compared.s_mean-(range*compared.s_mean))>comparing.s_mean)||((compared.v_mean-(range*compared.v_mean))>comparing.v_mean)) return false;
  if (((compared.h_mean+(range*compared.h_mean))<comparing.h_mean)||((compared.s_mean+(range*compared.s_mean))<comparing.s_mean)||((compared.v_mean+(range*compared.v_mean))<comparing.v_mean)) return false;
  
  //check maxDist
  //if ((compared.max_dist + (compared.max_dist*0.1)) > comparing.max_dist) return false;
  //if ((compared.max_dist - (compared.max_dist*0.1)) > comparing.max_dist) return false;

  return true;
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