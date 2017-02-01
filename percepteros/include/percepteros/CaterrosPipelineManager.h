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
  ros::Publisher desig_pub_;
  ros::ServiceServer service, singleService, setContextService, jsonService;

  const bool waitForServiceCall_;
  const bool useVisualizer_;
  bool useIdentityResolution_;
  bool pause_;


  std::mutex processing_mutex_;

  rs::Visualizer visualizer_;

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
    setContextService = nh_.advertiseService("set_context", &CaterrosPipelineManager::resetAECallback, this);
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
      std::string configFile_ = ros::package::getPath("percepteros") +"/config/cake.yaml";
      cv::FileStorage fs(configFile_, cv::FileStorage::READ);
      std::vector<std::string> lowLvlPipeline;
      fs["annotators"] >> lowLvlPipeline;
      engine.setNextPipeline(lowLvlPipeline);
      engine.applyNextPipeline();
      return true;
  }



  bool resetAECallback(suturo_perception_msgs::RunPipeline::Request &req,
                       suturo_perception_msgs::RunPipeline::Response &res){
      std::vector<std::string> objects = req.objects;
      std::string ctxName = "";
      for(std::string object : objects){
          if(object.compare("cake") == 0){
            ctxName="cakeros";
          } else {
            ctxName = "cylinder";
          }
      }
      if(resetAE(ctxName))
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

  bool resetAE(std::string newContextName);



  inline void setUseIdentityResolution(bool useIdentityResoltuion)
  {
    useIdentityResolution_ = useIdentityResoltuion;
  }
};

#endif
