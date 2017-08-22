//uima
#include <uima/api.hpp>

//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>
#include <rs/utils/common.h>

//percepteros
#include <percepteros/types/all_types.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>

//c++ base
#include <float.h>
#include <ctime>
#include <iostream>
#include <locale>

using namespace uima;
using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PCT;

typedef pcl::PointXYZ PointG;

class DataAquisition : public DrawingAnnotator
{
	private:
		PCT::Ptr cloud = PCT::Ptr(new PCT);
		PCT::Ptr cluster = PCT::Ptr(new PCT);
		PointG averageP, oneP, twoP, threeP, fourP;
		float one_x, one_y, one_z, two_x, two_y, two_z, three_x, three_y, three_z, four_x, four_y, four_z;
		float one_dist = FLT_MAX, two_dist = FLT_MAX, three_dist = FLT_MAX, four_dist = FLT_MAX;
		string one, two, three, four, dir;
		pcl::ExtractIndices<PointT> extract;

	public:
		int one_i = 0, two_i = 0, three_i = 0, four_i = 0;

	DataAquisition(): DrawingAnnotator(__func__){
	}

	TyErrorId initialize(AnnotatorContext &ctx)
	{
		outInfo("initialize");
		
		//extract first region parameters
		ctx.extractValue("oneX", one_x);
		ctx.extractValue("oneY", one_y);
		ctx.extractValue("oneZ", one_z);
		ctx.extractValue("oneName", one);

		//extract second region parameters
		ctx.extractValue("twoX", two_x);
		ctx.extractValue("twoY", two_y);
		ctx.extractValue("twoZ", two_z);
		ctx.extractValue("twoName", two);

		//extract third region parameters
		ctx.extractValue("threeX", three_x);
		ctx.extractValue("threeY", three_y);
		ctx.extractValue("threeZ", three_z);
		ctx.extractValue("threeName", three);

		//extract fourth region parameters
		ctx.extractValue("fourX", four_x);
		ctx.extractValue("fourY", four_y);
		ctx.extractValue("fourZ", four_z);
		ctx.extractValue("fourName", four);

		//get directory to save points to
		ctx.extractValue("dir", dir);

		return UIMA_ERR_NONE;
	}

	TyErrorId destroy()
	{
		outInfo("destroy");
		return UIMA_ERR_NONE;
	}

	TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
	{
		outInfo("data aquisition start\n");
		//get clusters
		rs::SceneCas cas(tcas);
		rs::Scene scene = cas.getScene();
		std::vector<rs::Cluster> clusters;
		scene.identifiables.filter(clusters);

		//clear clouds
		cloud->clear();
		cluster->clear();

		//get scene points
		cas.get(VIEW_CLOUD, *cloud);

		//make points
		PointG oneP(one_x, one_y, one_z);
		PointG twoP(two_x, two_y, two_z);
		PointG threeP(three_x, three_y, three_z);
		PointG fourP(four_x, four_y, four_z);

		for (int i = 0; i < clusters.size(); i++) {
			averageP = getAverage(cloud, clusters[i]);

			//compare one
			if (pcl::geometry::distance(averageP, oneP) < one_dist) {
				one_dist = pcl::geometry::distance(averageP, oneP);
				one_i = i;
			}

			//compare two
			if (pcl::geometry::distance(averageP, twoP) < two_dist) {
				two_dist = pcl::geometry::distance(averageP, twoP);
				two_i = i;
			}

			//compare three
			if (pcl::geometry::distance(averageP, threeP) < three_dist) {
				three_dist = pcl::geometry::distance(averageP, threeP);
				three_i = i;
			}

			//compare four
			if (pcl::geometry::distance(averageP, fourP) < four_dist) {
				four_dist = pcl::geometry::distance(averageP, fourP);
				four_i = i;
			}
		}

		//log findings
		outInfo("Found the following clusters with distances:");
		outInfo("Region One: " << one_i << " / " << one_dist);
		outInfo("Region Two: " << two_i << " / " << two_dist);
		outInfo("Region Three: " << three_i << " / " << three_dist);
		outInfo("Region Four: " << four_i << " / " << four_dist);

		//write clusters
		extract.setInputCloud(cloud);

		std::time_t t = std::time(NULL);
    	char mbstr[20];
    	std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    	string time(mbstr);
		writeCloud(cluster, clusters[one_i], dir + one + "/" + time + ".pcd");
		writeCloud(cluster, clusters[two_i], dir + two + "/" + time + ".pcd");
		writeCloud(cluster, clusters[three_i], dir + three + "/" + time + ".pcd");
		writeCloud(cluster, clusters[four_i], dir + four + "/" + time + ".pcd");

		return UIMA_ERR_NONE;
	}

	PointG getAverage(PCT::Ptr cloud, rs::Cluster cluster) {
		//setup stuff
		pcl::CentroidPoint<PointT> centroid;
		PointG average;

		//add points from cluster
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);

		for (std::vector<int>::const_iterator pit = cluster_indices->indices.begin(); pit != cluster_indices->indices.end(); pit++) {
			centroid.add(cloud->points[*pit]);
		}

		//return centroid
		centroid.get(average);
		return average;
	}

	void writeCloud(PCT::Ptr clust, rs::Cluster cluster, string name) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);

		clust->clear();
		extract.setIndices(cluster_indices);
		extract.filter(*clust);

		pcl::io::savePCDFile(name, *clust);
	}

	void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
	}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(DataAquisition)
