#include <percepteros/CaterrosPipelineManager.h>


void CaterrosPipelineManager::run()
{
  for(; ros::ok();)
  {
    std::unique_lock<std::mutex> lock1(processing_mutex_);
    //lock1.lock();
    //processing_mutex_.lock();
    if(waitForServiceCall_ || pause_)
    {
      usleep(100000);
    }
    else
    {
      engine.process(true);
    }
    lock1.unlock();
    //processing_mutex_.unlock();
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

    std::lock_guard<std::mutex> lock(processing_mutex_);
    this->init(contextAEPath, configFile);
    //processing_mutex_.unlock();

    return true;
  }
  else
  {
    return false;
  }
}
