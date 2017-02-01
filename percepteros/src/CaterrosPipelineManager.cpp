#include <percepteros/CaterrosPipelineManager.h>


void CaterrosPipelineManager::run()
{
  for(; ros::ok();)
  {
    processing_mutex_.lock();
    if(waitForServiceCall_ || pause_)
    {
      usleep(100000);
    }
    else
    {
      engine.process(true);
    }
    processing_mutex_.unlock();
    ros::spinOnce();
  }
}

bool CaterrosPipelineManager::resetAE(std::string newContextName)
{
  std::string contextAEPath;
  if(rs::common::getAEPaths(newContextName, contextAEPath))
  {
    outInfo("Setting new context: " << newContextName);
    cv::FileStorage fs(configFile, cv::FileStorage::READ);
    std::vector<std::string> lowLvlPipeline;
    fs["annotators"] >> lowLvlPipeline;

    processing_mutex_.lock();
    this->init(contextAEPath, configFile);
    processing_mutex_.unlock();

    return true;
  }
  else
  {
    return false;
  }
}
