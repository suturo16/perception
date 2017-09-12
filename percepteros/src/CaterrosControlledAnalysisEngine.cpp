#include <percepteros/CaterrosControlledAnalysisEngine.h>
#include <percepteros/types/all_types.h>
#include <percepteros/ObjectRegionFilter.h>

/**
 * @brief CaterrosControlledAnalysisEngine::init Initialize The ControlledAnalysisEngine, using the given aefile
 * as a definition of the existing annotators and the lowLvlPipeline as the default pipeline
 * @param AEFile definition of the existing annotators
 * @param lowLvlPipeline the default pipeline
 */
void CaterrosControlledAnalysisEngine::init(const std::string &AEFile, const std::vector<std::string> &lowLvlPipeline)
{
  uima::ErrorInfo errorInfo;

  size_t pos = AEFile.rfind('/');
  outInfo("Creating analysis engine: " FG_BLUE << (pos == AEFile.npos ? AEFile : AEFile.substr(pos)));

  engine = uima::Framework::createAnalysisEngine(AEFile.c_str(), errorInfo);

  if(errorInfo.getErrorId() != UIMA_ERR_NONE)
  {
    outError("createAnalysisEngine failed." << errorInfo.asString());
    throw uima::Exception(errorInfo);
  }
  // RSPipelineManager rspm(engine);
  rspm = new RSPipelineManager(engine);
  std::vector<icu::UnicodeString> &non_const_nodes = rspm->getFlowConstraintNodes();

  outInfo("*** Fetch the FlowConstraint nodes. Size is: "  << non_const_nodes.size());
  for(int i = 0; i < non_const_nodes.size(); i++)
  {
    std::string tempString;
    non_const_nodes.at(i).toUTF8String(tempString);
    outInfo(tempString);
  }

  rspm->aengine->getNbrOfAnnotators();
  outInfo("*** Number of Annotators in AnnotatorManager: " << rspm->aengine->getNbrOfAnnotators());

  // After all annotators have been initialized, pick the default pipeline

  rspm->setDefaultPipelineOrdering(lowLvlPipeline);
  rspm->setPipelineOrdering(lowLvlPipeline);
  // Get a new CAS
  outInfo("Creating a new CAS");
  cas = engine->newCAS();

  if(cas == NULL)
  {
    outError("Creating new CAS failed.");
    engine->destroy();
    delete engine;
    engine = NULL;
    throw uima::Exception(uima::ErrorMessage(UIMA_ERR_ENGINE_NO_CAS), UIMA_ERR_ENGINE_NO_CAS, uima::ErrorInfo::unrecoverable);
  }

  outInfo("initialization done: " << name << std::endl
          << std::endl << FG_YELLOW << "********************************************************************************" << std::endl);
  currentAEName = AEFile;
}

/**
 * @brief CaterrosControlledAnalysisEngine::process Executes the pipeline once
 * @param reset_pipeline_after_process unused
 */
void CaterrosControlledAnalysisEngine::process(bool reset_pipeline_after_process){
    cas->reset();
    UnicodeString ustrInputText;
    ustrInputText.fromUTF8(name);
    cas->setDocumentText(uima::UnicodeStringRef(ustrInputText));
    try
       {
         uima::CASIterator casIter = engine->processAndOutputNewCASes(*cas);
         for(int i = 0; casIter.hasNext(); ++i)
         {
           uima::CAS &outCas = casIter.next();

           // release CAS
           outInfo("release CAS " << i);
           engine->getAnnotatorContext().releaseCAS(outCas);
         }
       }
     catch(const rs::FrameFilterException &)
     {
       // Nothing changed, time to do something else
     }
     catch(const rs::Exception &e)
     {
       outError("Exception: " << std::endl << e.what());
     }
     catch(const uima::Exception &e)
     {
       outError("Exception: " << std::endl << e);
     }
     catch(const std::exception &e)
     {
       outError("Exception: " << std::endl << e.what());
     }
     catch(...)
     {
       outError("Unknown exception!");
   }
  percepteros::PipelineAnnotation annotation = rs::create<percepteros::PipelineAnnotation>(*cas);
  annotation.pipelineID.set(this->pipelineID.c_str());
  rs::SceneCas sCas(*cas);
  rs::Scene scene = sCas.getScene();
  scene.identifiables.append(annotation);
  outInfo("Appended PipelineID to cluster");
  std::vector<percepteros::PipelineAnnotation> pipeline_spec;
  scene.identifiables.filter(pipeline_spec);
  outInfo("set pipeline to: " << pipeline_spec[0].pipelineID.get());
  PipelineIdentifikator::pipelineID = pipelineID;
}
