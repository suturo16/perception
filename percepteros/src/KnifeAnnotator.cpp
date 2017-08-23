//UIMA
#include <uima/api.hpp>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types_conversion.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

//ROS
#include <geometry_msgs/PoseStamped.h>

//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/DrawingAnnotator.h>

//SUTURO
#include <percepteros/types/all_types.h>

using namespace uima;

typedef pcl::PointXYZRGBA PointR;
typedef pcl::PointCloud<PointR> PCR;
typedef pcl::Normal Normal;
typedef pcl::PointCloud<Normal> PCN;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointN> PC;

class KnifeAnnotator : public DrawingAnnotator {
	private:
		PCR::Ptr cloud_r = PCR::Ptr(new PCR);
		PCN::Ptr cloud_n = PCN::Ptr(new PCN);
		PC::Ptr cloud = PC::Ptr(new PC);
		PC::Ptr blade = PC::Ptr(new PC);
		PC::Ptr rack = PC::Ptr(new PC);
		int HUE_UPPER_BOUND, HUE_LOWER_BOUND;
		pcl::ExtractIndices<PointN> ex;
		pcl::VoxelGrid<PointN> filter;

	public:
		tf::Vector3 x, y, z;
		PointN highest, lowest;

	KnifeAnnotator(): DrawingAnnotator(__func__){
	}

	TyErrorId initialize(AnnotatorContext &ctx) {
		outInfo("Initialize KnifeAnnotator");
			
		//extract color parameters
		ctx.extractValue("minHue", HUE_LOWER_BOUND);
		ctx.extractValue("maxHue", HUE_UPPER_BOUND);

		return UIMA_ERR_NONE;
	}

	TyErrorId destroy() {
		outInfo("Destroy KnifeAnnotator");
		return UIMA_ERR_NONE;
	}

	TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec) {
		outInfo("Start KnifeAnnotator");

		//clear clouds
		cloud_r->clear();
		cloud_n->clear();
		cloud->clear();
		blade->clear();
		rack->clear();

		//get scene points
		rs::SceneCas cas(tcas);
		rs::Scene scene = cas.getScene();
		cas.get(VIEW_CLOUD, *cloud_r);
		cas.get(VIEW_NORMALS, *cloud_n);
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*cloud_r, *temp);
		pcl::concatenateFields(*temp, *cloud_n, *cloud);
		
		//tool clusters
		std::vector<rs::Cluster> tool_clusters;
		std::vector<std::vector<percepteros::ToolObject>> tool_annotations;
		scene.identifiables.filter(tool_clusters, tool_annotations);

		//rack clusters
		std::vector<rs::Cluster> rack_clusters;
		std::vector<std::vector<percepteros::RackObject>> rack_annotations;
		scene.identifiables.filter(rack_clusters, rack_annotations);

		//helper bools
		bool foundRack = false;
		bool foundKnife = false;

		//get y vector from rack if present
		if (rack_clusters.size() > 0) {
			foundRack = true;
			outInfo("Found rack. Using average normal for y-Vector.");
			if (rack_clusters.size() > 1) {
				outInfo("Found multiple racks! Using first one for y-Vector.");
			}

			std::vector<float> yv = rack_annotations[0][0].normal.get();
			y.setX(yv[0]);
			y.setY(yv[1]);
			y.setZ(yv[2]);
		}

		//prepare helpers
		ex.setInputCloud(cloud);
		//filter.setLeafSize(0.01f, 0.01f, 0.01f);

		int cluster_index = 0;
		//search for knife
		if (tool_clusters.size() > 0) {
			for (int i = 0; i < tool_clusters.size(); i++) {
				percepteros::ToolObject tool = tool_annotations[i][0];
				if (tool.hue.get() > HUE_LOWER_BOUND && tool.hue.get() < HUE_UPPER_BOUND) {
					outInfo("Found knife cluster.");
					foundKnife = true;
					cluster_index = i;
					extractPoints(tool_clusters[i], blade);

					//extract vectors
					x = getX(blade);
					if (!foundRack) {
						y = getY(blade);
					}
					z = x.cross(y);
					y = z.cross(x);
					x.normalize(); y.normalize(); z.normalize();

					//check vectors and continue if wrong, otherwise continue
					if (std::isnan(x[0]) || std::isnan(x[1]) || std::isnan(x[2]) ||
						std::isnan(y[0]) || std::isnan(y[1]) || std::isnan(x[2]) ||
						std::isnan(z[0]) || std::isnan(z[1]) || std::isnan(z[2])) {
						outInfo("Found wrong orientation. Abort.");
						foundKnife = false;
					} else {
						break;
					}
				}
			}
		}

		//if we didn't find a knife, we return an error
		if (!foundKnife) {
			outInfo("No knife found.");
			return UIMA_ERR_NONE;
		}

		//Building Annotations for Cluster
		rs::PoseAnnotation poseA = rs::create<rs::PoseAnnotation>(tcas);
		percepteros::RecognitionObject recA = rs::create<percepteros::RecognitionObject>(tcas);

		//get transform of camera
		tf::StampedTransform camToWorld;
		camToWorld.setIdentity();
		if (scene.viewPoint.has()) {
			rs::conversion::from(scene.viewPoint.get(), camToWorld);
		}

		//get origin
		tf::Vector3 origin;
		origin.setValue(highest.x, highest.y, highest.z);		

		//build transform
		tf::Transform transform;
		transform.setOrigin(origin);

		tf::Matrix3x3 rot;
		rot.setValue(   x[0], y[0], z[0],
						x[1], y[1], z[1],
						x[2], y[2], z[2]);
		transform.setBasis(rot);

		//building Recognition Object
		recA.name.set("cakeKnife");
		recA.type.set(6);
		recA.width.set(0.28f);
		recA.height.set(0.056f);
		recA.depth.set(0.03f);

		//building Pose Annotation
		tf::Stamped<tf::Pose> camera(transform, camToWorld.stamp_, camToWorld.child_frame_id_);
		tf::Stamped<tf::Pose> world(camToWorld * transform, camToWorld.stamp_, camToWorld.frame_id_);

		poseA.source.set("KnifeAnnotator");
		poseA.camera.set(rs::conversion::to(tcas, camera));
		poseA.world.set(rs::conversion::to(tcas, world));
			
		tool_clusters[cluster_index].annotations.append(poseA);
		tool_clusters[cluster_index].annotations.append(recA);
		outInfo("Finished looking for knife.");
			
		return UIMA_ERR_NONE;
	}

	//finds points with greatest distance
	void setEndpoints(PC::Ptr blade) {
		PointN begin, end;
		std::vector<PointN> endpoints(2);
		int size = blade->size();   
		float currDistance = 0;
		
		for (int i = 0; i < size; i++) {
			begin = blade->points[i];
			for (int j = i+1; j < size; j++) {
				end = blade->points[j];
				if (pcl::geometry::distance(begin, end) > currDistance) {
					endpoints[0] = begin;
					endpoints[1] = end;
					currDistance = pcl::geometry::distance(begin, end);
				}
			}
		}

		if (endpoints[0].x + endpoints[0].y + endpoints[0].z < endpoints[1].x + endpoints[1].y + endpoints[1].z) {
			highest = endpoints[0];
			lowest = endpoints[1];
		} else {
			highest = endpoints[1];
			lowest = endpoints[0];
		}
	}

	//gets x vector of blade
	tf::Vector3 getX(PC::Ptr blade) {
		setEndpoints(blade);
		x.setValue(lowest.x - highest.x, lowest.y - highest.y, lowest.z - highest.z);
		return x;
	}

	//get y vector of blade
	tf::Vector3 getY(PC::Ptr object) {
		tf::Vector3 object_normal(0, 0, 0);
		PC::Ptr temp = PC::Ptr(new PC);
		std::vector<int> indices;
		pcl::removeNaNNormalsFromPointCloud(*object, *temp, indices);
		int size = temp->size();

		for (size_t i = 0; i < size; i++) {
			object_normal[0] -= temp->points[i].normal_x / size;
			object_normal[1] -= temp->points[i].normal_y / size;
			object_normal[2] -= temp->points[i].normal_z / size;
		}

		return object_normal;
	}

	//Extracts knife points and thins them
	void extractPoints(rs::Cluster cluster, PC::Ptr container) {
		pcl::PointIndices::Ptr cluster_indices(new pcl::PointIndices);
		rs::ReferenceClusterPoints clusterpoints(cluster.points());
		rs::conversion::from(clusterpoints.indices(), *cluster_indices);

		ex.setIndices(cluster_indices);
		ex.filter(*container);
		
		//filter.setInputCloud(container);
		//filter.filter(*container);
	}

	//fills visualizer with informative cloud of knife annotator
	void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun) {
		if (firstRun) {
			visualizer.addPointCloud(cloud_r, "scene points");
		} else {
			visualizer.updatePointCloud(cloud_r, "scene points");
			visualizer.removeAllShapes();
		}
					
		visualizer.addCone(getCoefficients(x, highest), "x");
		visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "x");
		
		visualizer.addCone(getCoefficients(y, highest), "y");
		visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "y");

		visualizer.addCone(getCoefficients(z, highest), "z");
		visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "z");
	}

	//gets the coefficients for a cone visualizing one of the vectors
	pcl::ModelCoefficients getCoefficients(tf::Vector3 axis, PointN highest) {
		pcl::ModelCoefficients coeffs;
		//point
		coeffs.values.push_back(highest.x);
		coeffs.values.push_back(highest.y);
		coeffs.values.push_back(highest.z);
		//direction
		coeffs.values.push_back(axis[0]);
		coeffs.values.push_back(axis[1]);
		coeffs.values.push_back(axis[2]);
		//radius
		coeffs.values.push_back(1.0f);

		return coeffs;
	}
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(KnifeAnnotator)