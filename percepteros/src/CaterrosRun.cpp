#include <stdio.h>
#include <sstream>
#include <errno.h>
#include <iostream>
#include <chrono>
#include <condition_variable>

#include <uima/api.hpp>
#include "uima/internal_aggregate_engine.hpp"
#include "uima/annotator_mgr.hpp"

#include <rs/utils/RSPipelineManager.h>
#include <rs/utils/common.h>
//#include <rs_queryanswering/RSControledAnalysisEngine.h>
#include <percepteros/CaterrosPipelineManager.h>

#include <ros/ros.h>
#include <ros/package.h>

#undef OUT_LEVEL
#define OUT_LEVEL OUT_LEVEL_DEBUG
void help()
{
  std::cout << "Usage: CaterrosRun [options] analysisEngine.xml [...]" << std::endl
            << "Options:" << std::endl
            << "  -wait If using piepline set this to wait for a service call" << std::endl
            << "  -cwa use the list of objects from [pkg_path]/config/config.yaml to set a closed world assumption"
            << "  -visualizer  Enable visualization" << std::endl
            << "  -save PATH   Path for storing images" << std::endl;
}

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    help();
    return 1;
  }
  ros::init(argc, argv, std::string("RoboSherlock_") + getenv("USER"));

    std::vector<std::string> args;
    args.resize(argc - 1);
    for(int argI = 1; argI < argc; ++argI)
    {
      args[argI - 1] = argv[argI];
    }
    bool useVisualizer = true;
    bool waitForServiceCall = false;
    bool useCWAssumption = false;
    bool useObjIDRes = false;
    std::string savePath = getenv("HOME");

    size_t argO = 0;
    for(size_t argI = 0; argI < args.size(); ++argI)
    {
      const std::string &arg = args[argI];

      if(arg == "-visualizer")
      {
        useVisualizer = true;
      }
      else if(arg == "-wait")
      {
        waitForServiceCall = true;
      }
      else if(arg == "-cwa")
      {
        useCWAssumption = true;
      }
      else if(arg == "-idres")
      {
        useObjIDRes = true;
      }
      else if(arg == "-save")
      {
        if(++argI < args.size())
        {
          //savePath = args[argI];
        }
        else
        {
          outError("No save path defined!");
          return -1;
        }
      }
      else
      {
        args[argO] = args[argI];
        ++argO;
      }
    }
    args.resize(argO);

    struct stat fileStat;
    if(stat(savePath.c_str(), &fileStat) || !S_ISDIR(fileStat.st_mode))
    {
      outError("Save path \"" << savePath << "\" does not exist.");
      return -1;
    }

    std::string analysisEngineFile;
    for(int argI = 0; argI < args.size(); ++argI)
    {
      const std::string &arg = args[0];

      if(!rs::common::getAEPaths(arg, analysisEngineFile))
      {
        outError("analysis engine \"" << arg << "\" not found.");
        return -1;
      }
    }
    std::string configFile = ros::package::getPath("percepteros") +"/config/config.yaml";
    outInfo(analysisEngineFile);
    ros::NodeHandle n("~");
    try
    {
      CaterrosPipelineManager manager(useVisualizer, savePath, waitForServiceCall, n);
      manager.setUseIdentityResolution(useObjIDRes);
      manager.pause();
      manager.init(analysisEngineFile, configFile);
      manager.run();
      manager.stop();
    }
    catch(const rs::Exception &e)
    {
      outError("Exception: " << std::endl << e.what());
      return -1;
    }
    catch(const uima::Exception &e)
    {
      outError("Exception: " << std::endl << e);
      return -1;
    }
    catch(const std::exception &e)
    {
      outError("Exception: " << std::endl << e.what());
      return -1;
    }
    catch(...)
    {
      outError("Unknown exception!");
      return -1;
    }
  return 0;

}
