#include <percepteros/CaterrosPipelineManager.h>


void CaterrosPipelineManager::run()
{
  for(; ros::ok();)
  {
    if(waitForServiceCall_ || pause_)
    {
      usleep(100000);
    }
    else
    {
      engine.process(true);
    }
    ros::spinOnce();
  }
}

bool CaterrosPipelineManager::resetAE(std::string newPipelineName)
{
  std::string contextAEPath;
  if(rs::common::getAEPaths(newPipelineName, contextAEPath))
  {
    outInfo("Setting new context: " << newPipelineName);
    cv::FileStorage fs(configFile, cv::FileStorage::READ);
    std::vector<std::string> lowLvlPipeline;
    fs["annotators"] >> lowLvlPipeline;

    this->init(contextAEPath, configFile);
    return true;
  }
  else
  {
    return false;
  }
}
