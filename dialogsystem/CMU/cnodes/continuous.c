
/* ====================================================================
 * Copyright (c) 2017 Franklin Kenghagho Kenfack.  All rights
 * reserved.*/


#include <boost/thread.hpp>
#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <string.h>
#include <assert.h>
#include <sys/select.h>
#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>

#include "pocketsphinx.h"
/* special headers. */


#include <pthread.h> 
#include <xmlrpc-c/base.h>
#include <xmlrpc-c/client.h>
#include "config.h"                           /* information about this build environment */
#include <fdsink.h>

//defined macros
#define NAME "Xmlrpc-c Test Client"
#define VERSION "1.0"
#define NBTHREADS 1400
#define BEAMSIZE  1400


using namespace std;

//global variables
static pthread_mutex_t mutex;
static int pip[NBTHREADS];
static int*  pipes[NBTHREADS];
int counter=0;
static char const *cfg;
static char* configDict[NBTHREADS];
static char* configLM[NBTHREADS];
static pthread_t threads[NBTHREADS];
static ps_decoder_t* psobj[NBTHREADS];
static cmd_ln_t* configobj[NBTHREADS];
static char* hypobj[NBTHREADS];
static int32 scobj[NBTHREADS];
static int32 firstscobj[BEAMSIZE];


//collecting parameter
int32 TRESHOLD;
int INDEX;
int PNBTHREADS;
int PBEAMSIZE;
string HOST;
int PORT;
string RPCPORT;
string DATAPATH;
string ASRCWD;





//convert string to char*

char* stconvert(string s){
     
         char* p=new char[s.length()+1];
        for(int i=0;i<s.length();i++)
           p[i]=s[i];
        p[s.length()]='\0';
        return p;
     
    
}


//convert int to char*

char* stconvert(int n){
        int N=1;
        int m=n;
        while(m/10>0){
          m/=10;
          N++;   
        }
         char* p=new char[N+1];
        p[N]='\0';
        for(int i=N-1;i>=0;i--){
           p[i]='0'+(n%10);
           n/=10;
        }
        return p;
     
    
}
/*************************************************************** Audiostream class*************************/

//Audiostream

class Audiostream{
    public:
      //audio stream
      Audiostream(int pip[])
      {
        //set pipe
        setpipe(pip);
        //bit rate 24Khz=192bits/s
        _bitrate = 192;
        // Need to encoding or publish raw wave data
         _format="S16LE";
        // The bitrate at which to encode the audio
        _bitrate=192;
        // only available for raw data
        
        _channels=1;
        _depth=16;
        _sample_rate=16000;
        // The destination of the audio
        std::string dst_type;
        dst_type="fdsink";
        // The source of the audio
        std::string source_type;
        source_type="tcpserversrc";//tcpserversrc
        std::string device;
        device="";
        //main loop
        _loop = g_main_loop_new(NULL, false);
        //empty pipeline
        _pipeline = gst_pipeline_new("ros_pipeline");
        //pipe line bus
        _bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
        //supervise bus
        gst_bus_add_signal_watch(_bus);
        //bus event handler
        g_signal_connect(_bus, "message::error",
                         G_CALLBACK(onMessage), this);
        //free bus
        g_object_unref(_bus);
        // We create the sink first, just for convenience
        _sink = gst_element_factory_make("multifdsink", "sink");
       
       
        //tcpserversrc
        _source = gst_element_factory_make("tcpserversrc", "source");  
        g_object_set (G_OBJECT (_source), "host", stconvert(HOST),NULL);
        g_object_set (G_OBJECT (_source), "port",PORT ,NULL);
       

       
          gboolean link_ok;
         //filter
        _filter = gst_element_factory_make("capsfilter", "filter");
        //bus format
          GstCaps *caps;
          caps = gst_caps_new_simple("audio/x-raw",
                                     "channels", G_TYPE_INT, _channels,
                                     "width",    G_TYPE_INT, _depth,
                                     "depth",    G_TYPE_INT, _depth,
                                     "rate",     G_TYPE_INT, _sample_rate,
                                     "signed",   G_TYPE_BOOLEAN, TRUE,
                                     "format",   G_TYPE_STRING, "S16LE",
                                     NULL);

          g_object_set( G_OBJECT(_filter), "caps", caps, NULL);
          gst_caps_unref(caps);
          gst_bin_add_many( GST_BIN(_pipeline), _source,_filter, _sink, NULL);
          link_ok = gst_element_link_many( _source,_filter, _sink, NULL);
       
        //check if pipeline correctly linked
        if (!link_ok) {
          cout<<endl<<"Unsupported media type."<<endl;
          exitOnMainThread(1);
        }
        //start the pipeline
        gst_element_set_state(GST_ELEMENT(_pipeline), GST_STATE_PLAYING);
        //streaming thread
        //adding descriptor to sink
        int i;
        for(i=0;i<PNBTHREADS;i++){
          g_signal_emit_by_name(_sink, "add", this->in[i], G_TYPE_NONE);
       }
        //start streaming
        _gst_thread = boost::thread( boost::bind(g_main_loop_run, _loop) );
      }
       //destructor
      ~Audiostream()
      {  int i;
        cout<<endl<<"*******************STREAM TERMINATES*******************!!!."<<endl;
        //quit thread
        g_main_loop_quit(_loop);
        //stop pipeline
        gst_element_set_state(_pipeline, GST_STATE_NULL);
        //discharge the pipeline
        gst_object_unref(_pipeline);
        //discharge threads
        g_main_loop_unref(_loop);
       //close pipes
        for(i=0;i<PNBTHREADS;i++){
           close(this->in[i]);
        }
       cout<<endl<<"*******************STREAM TERMINATED*******************!!!."<<endl;
      }
      //exit thread
      void exitOnMainThread(int code)
      {
        delete this;
        exit(code);
      }

      //signal handling functions
      static gboolean onMessage (GstBus *bus, GstMessage *message, gpointer userData)
      {
        Audiostream *server = reinterpret_cast<Audiostream*>(userData);
        GError *err;
        gchar *debug;

        gst_message_parse_error(message, &err, &debug);
        cout<<endl<<endl<<GST_MESSAGE_SRC_NAME(message)<<endl<<endl;
        cout<<endl<<endl<<"Error:"<<debug<<endl<<endl;
        cout<<endl<<endl<<gst_message_type_get_name(GST_MESSAGE_TYPE(message))<<endl<<endl;
        
        g_error_free(err);
        g_free(debug);
        g_main_loop_quit(server->_loop);
        server->exitOnMainThread(1);
        return FALSE;
      }

      //set pipe
      void setpipe(int in[]);
    private:
      //attributes
      boost::thread _gst_thread;
      GstElement *_pipeline, *_source, *_filter, *_sink, *_convert, *_encode, *_tee;
      GstBus *_bus;
      int _bitrate, _channels, _depth, _sample_rate;
      GMainLoop *_loop;
      std::string _format;
      guint8 *data; 
      int in[NBTHREADS];
};

void Audiostream::setpipe(int in[]){
    int flags,i;
    for(i=0;i<PNBTHREADS;i++){
      this->in[i]=in[i];
      //flags = fcntl(this->in[i], F_GETFL, 0);
      //fcntl(this->in[i], F_SETFL, flags | O_NONBLOCK);
    }
}
/*************************************************************** Audiostream class end *************************/



static void 
dieIfFaultOccurred (xmlrpc_env * const envP) {
    if (envP->fault_occurred) {
        fprintf(stderr, "ERROR: %s (%d)\n",
                envP->fault_string, envP->fault_code);
        exit(1);
    }
}



int  sendDataOut(char recognizedWord[]){
       /******************************RPC CLIENT*************************/

    xmlrpc_env env;
    xmlrpc_value * resultP;
    xmlrpc_int32 sum;
    const char *  serverUrl; 
    const char *  methodName = "on_word_recognized";
    string s="http://"+HOST+":"+RPCPORT+"/RPC2";
    serverUrl=stconvert(s);
   
    
    /* Initialize our error-handling environment. */
    xmlrpc_env_init(&env);

    /* Create the global XML-RPC client object. */
    xmlrpc_client_init2(&env, XMLRPC_CLIENT_NO_FLAGS, NAME, VERSION, NULL, 0);
    dieIfFaultOccurred(&env);

    printf("Making XMLRPC call to server url '%s' method '%s' "
           "to request the sum "
           "of 5 and 7...\n", serverUrl, methodName);

    /* Make the remote procedure call */
    resultP = xmlrpc_client_call(&env, serverUrl, methodName,
                                 "(s)", recognizedWord);
    dieIfFaultOccurred(&env);
    
    /* Get our sum and print it out. */
    xmlrpc_read_int(&env, resultP, &sum);
    dieIfFaultOccurred(&env);
    printf("The sum is %d\n", sum);
    
    /* Dispose of our result value. */
    xmlrpc_DECREF(resultP);

    /* Clean up our error-handling environment. */
    xmlrpc_env_clean(&env);
    
    /* Shutdown our XML-RPC client library. */
    xmlrpc_client_cleanup();

    return sum;

/*******************************END******************************/


}











static const arg_t cont_args_def[] = {
    POCKETSPHINX_OPTIONS,
    /* Argument file. */
    {"-argfile",
     ARG_STRING,
     NULL,
     "Argument file giving extra arguments."},
    {"-adcdev",
     ARG_STRING,
     NULL,
     "Name of audio device to use for input."},
    {"-infile",
     ARG_STRING,
     NULL,
     "Audio file to transcribe."},
    {"-inmic",
     ARG_BOOLEAN,
     "no",
     "Transcribe audio from microphone."},
    
    {"-time",
     ARG_BOOLEAN,
     "no",
     "Print word times in file transcription."},
    CMDLN_EMPTY_OPTION
};





/* Sleep for specified msec */

static void
sleep_msec(int32 ms)
{
    struct timeval tmo;
    tmo.tv_sec = 0;
    tmo.tv_usec = ms * 1000;
    select(0, NULL, NULL, NULL, &tmo);
}

/*
 *Recognition
 */

static bool isEmpty(char* s){
       if(s==NULL)
         return TRUE;
       else{
          int n=strlen(s);
          for(int i=0;i<n;i++)
             if(s[i]!=' ')
               return FALSE;
          return TRUE;
       }
 } 

static void recognize_from_microphone(ps_decoder_t* ps, cmd_ln_t* config, int i)
{
    
    int16 adbuf[2048];
    uint8 utt_started, in_speech;
    int32 k,score, posterior;
    char const *hyp;
    int length,j,max,l,flags;
    char finalhyp[1000];
    finalhyp[0]='\0';
    int inout;//pipes
    Audiostream* stream;
   
    if (ps_start_utt(ps) < 0)
        E_FATAL("Failed to start utterance\n");
    utt_started = FALSE;
    
     inout=pipes[i][0]; 
    //making non blocking stream
    flags = fcntl(inout, F_GETFL, 0);
    fcntl(inout, F_SETFL, flags | O_NONBLOCK);
    //audiostream output. only instanciated by thread 0
    if(i==0)
       stream=new Audiostream(pip);
      
    E_INFO("Ready....\n");
    for (;;) {
       //printf("...........NOTHING %d %d....\n",i,k);
        k = read(inout, adbuf, 4096);
        if(k==-1)
           k++;
        if (k < 0)
            E_FATAL("Failed to read audio\n");
       k=k/2;
        ps_process_raw(ps, adbuf, k, FALSE, FALSE);
        in_speech = ps_get_in_speech(ps);
        if (in_speech && !utt_started) {
            utt_started = TRUE;
            E_INFO("Listening...\n");
        }
       
        if (!in_speech && utt_started) {
            /* speech -> silence transition, time to start new utterance  */
            ps_end_utt(ps);
            hyp = ps_get_hyp(ps, &score );
            posterior=ps_get_prob(ps);
            //print full result for this thread
            printf(" Final score: %d  \n", score);
            printf(" Final posterior: %d  \n", posterior);
            //discard previous hypothesis
            if(hypobj[i]!=NULL)
            free(hypobj[i]);
            if(hyp!=NULL){
                    //allocate space for new hypothesis
                    length=strlen(hyp)+1;
                    hypobj[i]=(char*)malloc(sizeof(char)*length);
                    //save hypothesis
                    strcpy(hypobj[i],hyp);
                    //save score;
                    scobj[i]=score; 
                    if(scobj[i]<TRESHOLD) 
                    scobj[i]=0;
                    //print hyp
                    printf("\n\n HYPOTHESIS %d %d: %s \n\n",i,scobj[i], hyp);
            }else
                    scobj[i]=0;

            pthread_mutex_lock(&mutex);
            printf("\n\n------------------ THREAD %d,%d------------\n\n",i,counter);
            //critical section: shared ressource: counter
            counter++;
            if(counter==PNBTHREADS){
            //all threads have completed.
              counter=0;
            //get the BEAMSIZE best hypothesis
              for(l=0;l<PBEAMSIZE;l++){
                 max=0;
                 for(j=0;j<PNBTHREADS;j++)
                   if(scobj[j]>scobj[max] && scobj[j]<0 || scobj[j]<0 && scobj[max]>=0)
                     max=j;
                 firstscobj[l]=max;
                 if(scobj[max]<0)
                   scobj[max]=1;
              }
                 
              //merge the BEAMSIZE best hypothesis
              for(j=0;j<PBEAMSIZE;j++)
                 
                  if(scobj[firstscobj[j]]==1){
                          strcat(finalhyp,hypobj[firstscobj[j]]);
                          strcat(finalhyp," ");
                  }
              if(strlen(finalhyp)>0){
                if(!isEmpty(finalhyp)){
		        printf("\n\n HYPOTHESIS: %s\n\n",finalhyp );
		        if(sendDataOut(finalhyp)==0){
		           printf("\n\n---HYPOTHESIS SENT SUCCESSFULLY-\n\n");
		        }else
		         printf("\n\n---HYPOTHESIS SENDING FAILED-\n\n");
		       
                
              }else
                  printf("\n\n HYPOTHESIS: NOTHING\n\n");
               finalhyp[0]='\0';
            }}
            fflush(stdout);//output console  update
            printf("\n\n-----------------------------------------\n\n");
            //exit critical section
            pthread_mutex_unlock(&mutex);
            
            if (ps_start_utt(ps) < 0)
                E_FATAL("Failed to start utterance\n");
            utt_started = FALSE;
            E_INFO("Ready....\n");
        }
        sleep_msec(100);
    }
    close(inout);
}


void *recognizer(void*arg)
{



     int i;
     i=*((int*)arg);
     configobj[i]= cmd_ln_init(NULL,  cont_args_def, TRUE, "-dict", configDict[i],"-lm", configLM[i], "-inmic", "yes",NULL);
    
     /* Handle argument file as -argfile. */
    if (configobj[i] && (cfg = cmd_ln_str_r(configobj[i], "-argfile")) != NULL) {
        configobj[i] = cmd_ln_parse_file_r(configobj[i], cont_args_def, cfg, FALSE);
    }

    if (configobj[i] == NULL || (cmd_ln_str_r(configobj[i], "-infile") == NULL && cmd_ln_boolean_r(configobj[i], "-inmic") == FALSE)) {
	E_INFO("Specify '-infile <file.wav>' to recognize from file or '-inmic yes' to recognize from microphone.\n");
        cmd_ln_free_r(configobj[i]);
	return (void*)1;
    }

    ps_default_search_args(configobj[i]);
    psobj[i] = ps_init(configobj[i]);
    if (psobj[i] == NULL) {
        cmd_ln_free_r(configobj[i]);
        return (void*)1;
    }

  

    if (cmd_ln_str_r(configobj[i], "-infile") != NULL) {
        //recognize_from_file(psobj[i],configobj[i]);
    } else if (cmd_ln_boolean_r(configobj[i], "-inmic")) {
        recognize_from_microphone(psobj[i],configobj[i],i);
    }

    ps_free(psobj[i]);
    cmd_ln_free_r(configobj[i]);

    return (void*)0;

}











/********************** MAIN THREADS ***************************************************/


int
main(int argc, char *argv[])
{
 //check all parameters are there
 if(argc!=10){
   cout<<endl<<endl<<"Error in sphinx asr: incorrect parameters..."<<endl<<endl;
   return 0;
 
 }else{

   //collecting parameter
   cout<<endl<<"Loading parameters..."<<endl;
   INDEX=atoi(argv[1]);
   cout<<endl<<"INDEX: "<<INDEX<<endl;
   PNBTHREADS=atoi(argv[2]);
   cout<<endl<<"NBTHREADS: "<<PNBTHREADS<<endl;
   PBEAMSIZE=atoi(argv[3]);
   cout<<endl<<"BEAMSIZE: "<<PBEAMSIZE<<endl;
   HOST=string(argv[4]);
   cout<<endl<<"LOCALHOST: "<<HOST<<endl;
   PORT=atoi(argv[5]);
   cout<<endl<<"LOCALPORT: "<<PORT<<endl;
   DATAPATH=string(argv[6]);
   cout<<endl<<"DATAPATH: "<<DATAPATH<<endl;
   ASRCWD=string(argv[7]);
   cout<<endl<<"ASRCWD: "<<ASRCWD<<endl;
   RPCPORT=string(argv[8]);
   cout<<endl<<"RPCPORT: "<<RPCPORT<<endl;
   TRESHOLD=atoi(argv[9]);
   cout<<endl<<"TRESHOLD: "<<TRESHOLD<<endl;
   //create configuration
   int i;
   for(i=0;i<PNBTHREADS;i++){
      configDict[i]=stconvert(ASRCWD+"/"+DATAPATH+"/pepper"+string(stconvert(INDEX+i))+".dic");
      cout<<endl<<"DIC: "<<configDict[i]<<endl;
      configLM[i]=stconvert(ASRCWD+"/"+DATAPATH+"/pepper"+string(stconvert(INDEX+i))+".lm");
      cout<<endl<<"LM: "<<configLM[i]<<endl;
   }

   
   int t=0,rc;
   strlen(NULL);
   //init gstreamer
   gst_init(&argc, &argv);
   //create pipes
   for(t=0;t<PNBTHREADS;t++){
      pipes[t]=new int[2];
      pipe(pipes[t]);
      pip[t]=pipes[t][1];
   }
   
   //create threads
   pthread_mutex_init(&mutex, NULL);
   for(t=0; t<PNBTHREADS; t++){
       printf("In main: creating thread %d\n", t);
       rc = pthread_create(&threads[t], NULL, recognizer, (void *)(new int(t)));
       if (rc){
          printf("ERROR; return code from pthread_create() is %d\n", rc);
          exit(-1);
       }
    }

  //wait until all joining threads terminate
  pthread_exit(NULL);
  //destroys pipes
  for(t=0;t<PNBTHREADS;t++){
    close(pipes[t][0]);
    close(pipes[t][1]);
  }
  //destroy mutex
  pthread_mutex_destroy(&mutex);

  return 0;  
}
}
