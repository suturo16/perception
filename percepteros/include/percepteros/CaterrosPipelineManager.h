#ifndef CATERROSPIPELINEMANAGER_H
#define CATERROSPIPELINEMANAGER_H

#include <rs/utils/RSAnalysisEngineManager.h>
#include <suturo_perception_msgs/RunPipeline.h>
#include <percepteros/CaterrosControlledAnalysisEngine.h>
#include <ros/ros.h>


class CaterrosPipelineManager
{

private:

  CaterrosControlledAnalysisEngine engine;
  ros::NodeHandle nh_;
  bool waitForServiceCall_;
  rs::Visualizer visualizer_;
  bool useVisualizer_;
  bool useIdentityResolution_;
  bool pause_;

  ros::Publisher desig_pub_;
  ros::ServiceServer service, singleService, setContextService, jsonService;

  std::mutex processing_mutex_;

  std::string configFile;
  std::vector<std::string> lowLvlPipeline_;
	

public:

  CaterrosPipelineManager(const bool useVisualizer, const std::string &savePath,
                   const bool &waitForServiceCall, ros::NodeHandle n):
    engine(n), nh_(n), waitForServiceCall_(waitForServiceCall), visualizer_(savePath),
    useVisualizer_(useVisualizer), useIdentityResolution_(false), pause_(true)
  {

    outInfo("Creating resource manager"); // TODO: DEBUG
    uima::ResourceManager &resourceManager = uima::ResourceManager::createInstance("RoboSherlock"); // TODO: change topic?

    switch(OUT_LEVEL)
    {
    case OUT_LEVEL_NOOUT:
    case OUT_LEVEL_ERROR:
      resourceManager.setLoggingLevel(uima::LogStream::EnError);
      break;
    case OUT_LEVEL_INFO:
      resourceManager.setLoggingLevel(uima::LogStream::EnWarning);
      break;
    case OUT_LEVEL_DEBUG:
      resourceManager.setLoggingLevel(uima::LogStream::EnMessage);
      break;
    }


    // Call this service to switch between AEs
    //setContextService = nh_.advertiseService("set_context", &CaterrosPipelineManager::resetAECallback, this);
    setContextService = nh_.advertiseService("set_pipeline", &CaterrosPipelineManager::setPipelineCallback, this);


  }
  ~CaterrosPipelineManager()
  {
    uima::ResourceManager::deleteInstance();
    outInfo("RSControledAnalysisEngine Stoped");
  }

  /*brief
   * init the AE Manager
   **/
  void init(std::string &xmlFile, std::string &configFile)
  {
    this->configFile = configFile;
    cv::FileStorage fs(configFile, cv::FileStorage::READ);
    if(lowLvlPipeline_.empty()) //if not set programatically, load from a config file
    {
      fs["annotators"] >> lowLvlPipeline_;
    }
    engine.init(xmlFile, lowLvlPipeline_);
    if(useVisualizer_)
    {
      visualizer_.start();
    }
  }

  /* brief
   * run the AE in the manager
   */
  void run();

  void stop()
  {
    /*engine.resetCas();
    engine.stop();*/
  }

  inline void pause()
  {
    processing_mutex_.lock();
    pause_ = !pause_;
    processing_mutex_.unlock();
  }

  bool setPipelineCallback(suturo_perception_msgs::RunPipeline::Request &req,
                       suturo_perception_msgs::RunPipeline::Response &res){
	waitForServiceCall_ = false;
      std::vector<std::string> objects = req.objects;
      std::string pipelineName = "";
      //TODO multiple objects can be given, decide on annotators depending on given objects
      for(std::string object : objects){
          if(object == "cake"){
            pipelineName = "cake";
          } else if(object == "cylinder"){
            pipelineName = "cylinder";
          } else if(object =="knife"){
            pipelineName = "knife";
          } else if (object == "end") {
			      pipelineName = "end";
          } else if (object == "spatula") {
            pipelineName = "spatulaRecognition";
          } else if (object == "plate") {
            pipelineName = "plate";
          } else if (object == "board") {
            pipelineName = "board";
		      } else {
            outInfo("No Corresponding Object found, setting pipelineName to 'config'!");
            pipelineName= "config";
          }
      }
      std::string configFile_ = ros::package::getPath("percepteros") +"/config/"+pipelineName+".yaml";
      cv::FileStorage fs(configFile_, cv::FileStorage::READ);
      std::vector<std::string> lowLvlPipeline;
      fs["annotators"] >> lowLvlPipeline;
      engine.setNextPipeline(lowLvlPipeline, pipelineName);
      return true;
  }

  bool resetAECallback(suturo_perception_msgs::RunPipeline::Request &req,
                       suturo_perception_msgs::RunPipeline::Response &res){
      std::vector<std::string> objects = req.objects;
      std::string pipelineName = "";
      for(std::string object : objects){
          if(object.compare("cake") == 0){
            pipelineName="cake";
          } else if(object.compare("cylinder") == 0){
            pipelineName = "cylinder";
          } else {
              pipelineName= "config";
          }
      }
      if(resetAE(pipelineName))
      {
        return true;
      }
      else
      {
        outError("Contexts need to have a an AE defined");
        outInfo("releasing lock");
        return false;
      }

      return true;
  }

  bool resetAE(std::string newPipelineName);



  inline void setUseIdentityResolution(bool useIdentityResoltuion)
  {
    useIdentityResolution_ = useIdentityResoltuion;
  }
};

#endif
