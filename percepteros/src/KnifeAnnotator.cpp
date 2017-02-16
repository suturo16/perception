#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/common.h>

#include <pcl/features/normal_3d.h>
#include <percepteros/types/all_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace uima;

typedef pcl::PointNormal PointG;
typedef pcl::Normal PointN;
typedef pcl::PointXYZ PointT;

class KnifeAnnotator : public DrawingAnnotator
{
private:
  pcl::PointCloud<PointT>::Ptr cloud_ptr;
	pcl::PointCloud<PointN>::Ptr normal_ptr;
	pcl::PointCloud<PointG>::Ptr cloud_normal_ptr;
	pcl::PointCloud<PointG>::Ptr model_ptr;

public:

  KnifeAnnotator(): DrawingAnnotator(__func__){
      cloud_ptr = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
			normal_ptr = pcl::PointCloud<PointN>::Ptr(new pcl::PointCloud<PointN>);
			cloud_normal_ptr = pcl::PointCloud<PointG>::Ptr(new pcl::PointCloud<PointG>);
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
    rs::SceneCas cas(tcas);
		
		//get points and normals
    cas.get(VIEW_CLOUD, *cloud_ptr);
    cas.get(VIEW_NORMALS, *normal_ptr);
		
		//get points from ply file
		model_ptr = loadPly("../data/knife.ply");
    return UIMA_ERR_NONE;
  }

	pcl::PointCloud<PointG>::Ptr loadPly(const char* filename) {
		pcl::PointCloud<PointG>::Ptr model = pcl::PointCloud<PointG>::Ptr(new pcl::PointCloud<PointG>);
		int numVertices = 0;

		std::ifstream ifs(filename);
		
		//check if file available
		if(!ifs.is_open()) {
			printf("Cannot open file\n");
			return model;
		}
		
		//get number of vertices
		std::string str;
		while (str.substr(0, 10) != "end_header") {
			std::string entry = str.substr(0, 14);
			if (entry == "element vertex") {
				numVertices = atoi(str.substr(15, str.size()-15).c_str());
			}
			std::getline(ifs, str);
		}

		//get data of points
		for (int i = 0; i < numVertices; i++) {
			float data[6];
			PointG* point = new PointG;
			
			//get raw data
			for (int j = 0; j < 6; j++) {
				std::getline(ifs, str, ' ');
				data[j] = stof(str);
			}
			std::getline(ifs, str);

			//normalize normals
			double norm = sqrt(data[3]*data[3] + data[4]*data[4] + data[5]*data[5]);
			if (norm > 0.00001) {
				data[3] /= (float)norm;
				data[4] /= (float)norm;
				data[5] /= (float)norm;
			}
			
			//setting point
			point->x = data[0];
			point->y = data[1];
			point->z = data[2];
			point->normal_x = data[3];
			point->normal_y = data[4];
			point->normal_z = data[5];
			
			//push point
			model->push_back(*point);
		}

		return model;
	}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(KnifeAnnotator)
