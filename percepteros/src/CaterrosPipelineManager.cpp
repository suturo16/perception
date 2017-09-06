#include <percepteros/CaterrosPipelineManager.h>

/**
 * @brief CaterrosPipelineManager::run Runs the pipeline. Checks first, whether a new pipeline was selected and
 * applies the new pipeline if needed.
 */
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
      if(engine.queued){
        engine.applyNextPipeline();
      }
      engine.process(true);
    }
    ros::spinOnce();
  }
}

/**
 * @brief CaterrosPipelineManager::resetAE Resets the AnalysisEngine.
 * @param newPipelineName The name of the new AnalysisEngine
 * @return
 */
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
