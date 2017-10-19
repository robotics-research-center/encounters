#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "highgui.h"
#include "opencv2/opencv.hpp"
#include <string>
#include <fstream>
#include <sstream> 
#include <iostream>
#include <pthread.h>

#include "includes/helperfunctions.h"

// DLoopDetector and DBoW2
#include <DBoW2/DBoW2.h> // defines Surf64Vocabulary
#include "DLoopDetector.h" // defines Surf64LoopDetector
#include <DUtilsCV/DUtilsCV.h> // defines macros CVXX


#include <assert.h>     /* assert */

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
// Demo
#include "includes/demoDetector.h"


using namespace DLoopDetector;
using namespace DBoW2;
using namespace std;
using namespace cv;

#define PI 3.14159265




//HAVE TO DECIDE NODE IDS




// Directory of Images
string IMG_DIR1 = "";
string IMG_DIR2 = "";
// DLoop resources
static const string VOC_FILE = "./resources/huskymerge_voc.voc.gz";

    string g2ofilename_master = "session1-nfg.g2o"; //file like offline.g2o
    string g2ofilename_client = "session2-nfg.g2o";
    string arfilename_master = "AR-session1.txt";
    string arfilename_client = "AR-session2.txt";

static const int IMAGE_W = 640; // image size
static const int IMAGE_H = 480;
gtsam::NonlinearFactorGraph nfg[2];

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
void loadLoop(gtsam::NonlinearFactorGraph &dataset, gtsam::ISAM2 &isam2);
gtsam::Pose3 getPose(float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22, float x, float y, float z, int systemcase);
/// This functor extracts SURF64 descriptors in the required format


struct for_ar_thread
{
  int numimages;
  bool master=false;
  gtsam::ISAM2 *isam2;
  gtsam::NonlinearFactorGraph *nfg_final;
  std::vector<bool> *detected;
  std::vector<gtsam::Pose3> *ARpose;
};



void *ar_thread(void *t)
{
   for_ar_thread* obj = (for_ar_thread *) t;
   if(obj->master){
   cout << "entering master thread!!" << endl;
   }
   else{
   cout << "entering client thread!!" << endl;
   }


   cout << "bool value: " << obj->master << endl;

   // Calling ar function
   if(obj->master){
	std::cout<<"MASTER in if"<<std::endl;
   master_AR(obj->numimages,*(obj->isam2),*(obj->nfg_final),*(obj->detected),*(obj->ARpose));
   }
   else{
	std::cout<<"CLIENT in if"<<std::endl;
   client_AR(obj->numimages,*(obj->isam2),*(obj->nfg_final),*(obj->detected),*(obj->ARpose));
   }


   if(obj->master){
   cout << "exiting master thread!!" << endl;
   }
   else{
   cout << "exiting client thread!!" << endl;
   }

   return (void *)obj;
}




// For single loop
int main()
{

/**
    std::ifstream myfile("/home/tushar/codes/robotEncounters/AR.txt");
    //myfile.open();
    int detected,j;
    double dist,x,y,z,rot11,rot12,rot13,rot21,rot22,rot23,rot31,rot32,rot33;
    int counter=0;
    for(int i=0;i<2500;i++){
        myfile>>j>>detected;
        if(detected==1){
            if(counter>=5){
                cout<<"counter: "<<counter<<endl;
                break;
            }
            counter++;
            myfile>>dist>>x>>y>>z>>rot11>>rot12>>rot13>>rot21>>rot22>>rot23>>rot31>>rot32>>rot33;
            gtsam::Rot3 rot1(rot11,rot12,rot13,rot21,rot22,rot23,rot31,rot32,rot33);
            double thetay = atan2(rot13, sqrt(rot11*rot11 + rot12*rot12));

            double thetax = atan2(-rot23, rot33);

            double thetaz = atan2(-rot12, rot11);

            double thetay0 = atan2(-rot31, sqrt(rot32*rot32 + rot33*rot33));

            double thetax0 = atan2(rot32, rot33);

            double thetaz0 = atan2(rot21, rot11);
            cout<<"j: "<<j<<endl;
            cout<<"thetax: "<< (thetax > 0 ? thetax : (2*PI + thetax)) * 360 / (2*PI) << endl;
            cout<<"thetay: "<< (thetay > 0 ? thetay : (2*PI + thetay)) * 360 / (2*PI) << endl;
            cout<<"thetaz: "<< (thetaz > 0 ? thetaz : (2*PI + thetaz)) * 360 / (2*PI) << endl;

            cout<<"thetax0: "<< (thetax0 > 0 ? thetax0 : (2*PI + thetax0)) * 360 / (2*PI) << endl;
            cout<<"thetay0: "<< (thetay0 > 0 ? thetay0 : (2*PI + thetay0)) * 360 / (2*PI) << endl;
            cout<<"thetaz0: "<< (thetaz0 > 0 ? thetaz0 : (2*PI + thetaz0)) * 360 / (2*PI) << endl;
        }
    }



    myfile.close();
    cout<<"Theta done!"<<endl;

    return 0;

**/



    gtsam::ISAM2Params params;
    params.optimizationParams = gtsam::ISAM2DoglegParams();
    params.relinearizeSkip = 1;
    params.enablePartialRelinearizationCheck = true;
    gtsam::ISAM2 isam2_master(params);
    gtsam::ISAM2 isam2_client(params);
    gtsam::NonlinearFactorGraph nfg_master;
    gtsam::NonlinearFactorGraph nfg_client;
	gtsam::NonlinearFactorGraph nfg_temp;
    std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr> data_master = gtsam::load3D(g2ofilename_master);
    std::pair<gtsam::NonlinearFactorGraph::shared_ptr, gtsam::Values::shared_ptr> data_client = gtsam::load3D(g2ofilename_client);

    nfg[0] = *(data_master.first);
    nfg_temp = *(data_client.first);
	int nfgsize=nfg_temp.size();
	gtsam::Matrix M(6, 6); 
	M<<1000,0,0,0,0,0,0,1000,0,0,0,0,0,0,1000,0,0,0,0,0,0,1000,0,0,0,0,0,0,1000,0,0,0,0,0,0,1000;
	gtsam::SharedNoiseModel model;
	model = gtsam::noiseModel::Gaussian::Information(M, true);
	for(int g=0;g<nfgsize;g++){
		gtsam::NonlinearFactor::shared_ptr factor = nfg_temp[g];
    	gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr observation = 
        		boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);

		gtsam::NonlinearFactor::shared_ptr factor2(
		  new gtsam::BetweenFactor<gtsam::Pose3>(10000+observation->key1(), 10000+observation->key2(), observation->measured(), model)); //get_noiseModel
		//factor2->print("");

		nfg[1].push_back(factor2);
	}

//check this again

    std::cout<<"dataset size master: "<<nfg[0].size()<<std::endl;
    std::cout<<"dataset size client: "<<nfg[1].size()<<std::endl;


  // Number of images in given directory
  int numImages1 =2534;
/**
  std::string path1 = IMG_DIR1 + "/left/";
  char * dst1 = new char[path1.length() + 1];
  std::strcpy(dst1,path1.c_str());

  numImages1 = listdir(dst1);
**/
  int numImages2 =2681;
/**
  std::string path2 = IMG_DIR2 + "/left/";
  char * dst2 = new char[path2.length() + 1];
  std::strcpy(dst2,path2.c_str());

  numImages2 = listdir(dst2);
**/



//-------------------------------------------------------

  std::vector<bool> detected1;
  std::vector<gtsam::Pose3> ARpose1;
  std::vector<bool> detected2;
  std::vector<gtsam::Pose3> ARpose2;


  string line;
  ifstream myfile1(arfilename_master);
  if (myfile1.is_open())
  {
    int i=0;
    while ( getline(myfile1,line) )
    {    
	std::istringstream in(line);

	int frame;
	float dist;
	int tagId;
	float r00,r01,r02,r10,r11,r12,r20,r21,r22,x,y,z;

	in >> frame >> tagId;
	if(tagId==-1)
	    {
		detected1.push_back(false);
		ARpose1.push_back(gtsam::Pose3());
		continue; }
	detected1.push_back(true);
        in >> dist >> x >> y >> z >> r00 >> r01 >> r02 >> r10 >> r11 >> r12 >> r20 >> r21 >> r22 ;  
	gtsam::Pose3 p=getPose(r00,r01,r02,r10,r11,r12,r20,r21,r22,x,y,z,1);
	ARpose1.push_back(p);
    }
    myfile1.close();
  }

  assert(detected1.size()==numImages1);




  ifstream myfile2(arfilename_client);
  if (myfile2.is_open())
  {
    int i=0;
    while ( getline (myfile2,line) )
    {    
	std::istringstream in(line);

	int frame;
	float dist;
	int tagId;
	float r00,r01,r02,r10,r11,r12,r20,r21,r22,x,y,z;

	in >> frame >> tagId;
	if(tagId==-1)
	    {
		detected2.push_back(false);
		ARpose2.push_back(gtsam::Pose3());
		continue; }
	detected2.push_back(true);
        in >> dist >> x >> y >> z >> r00 >> r01 >> r02 >> r10 >> r11 >> r12 >> r20 >> r21 >> r22 ;  
	gtsam::Pose3 p=getPose(r00,r01,r02,r10,r11,r12,r20,r21,r22,x,y,z,1);
	ARpose2.push_back(p);
    }
    myfile2.close();
  }

  assert(detected2.size()==numImages2);
  detected1.resize(detected1.size());
  detected2.resize(detected2.size());
  ARpose1.resize(ARpose1.size());
  ARpose2.resize(ARpose2.size());






  pthread_t thread[2];
  pthread_attr_t attr;

  /* Initialize and set thread detached attribute */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  for_ar_thread arthread1;
  for_ar_thread arthread2;


  arthread1.numimages = numImages1;
  arthread1.master = true;
  arthread1.isam2 = &isam2_master;
  arthread1.nfg_final = &nfg_master;
  arthread1.detected = &detected1;
  arthread1.ARpose = &ARpose1;

  arthread2.numimages = numImages2;
  arthread2.isam2 = &isam2_client;
  arthread2.nfg_final = &nfg_client;
  arthread2.detected = &detected2;
  arthread2.ARpose = &ARpose2;

  pthread_create(&thread[0], &attr, ar_thread, (void *)&arthread1); //master
  pthread_create(&thread[1], &attr, ar_thread, (void *)&arthread2); //client
  
  // Wait for Completion
  pthread_join(thread[0],NULL);
  pthread_join(thread[1],NULL);



/**
  const string outputfile="isam.g2o";
  const string outputfile2="isamnfg.g2o";
  const string outputfile3="nfg.g2o";
  const string outputfile4="unopt-batch.g2o";
    gtsam::Values estimate2(isam2.calculateEstimate());
    gtsam::NonlinearFactorGraph datasetempty;
    gtsam::writeG2o(datasetempty,estimate2,outputfile);
    gtsam::writeG2o(nfg,gtsam::Values(),outputfile3);
    gtsam::writeG2o(nfg,estimate2,outputfile2);
    gtsam::writeG2o(nfg,initialunopt,outputfile4);
  cout << "done" << endl;
**/

  return 0;
}

gtsam::Pose3 getPose(float r00, float r01, float r02, float r10, float r11, float r12, float r20, float r21, float r22, float x, float y, float z, int systemcase){

gtsam::Pose3 po;

switch(systemcase) {
   case 1  :
	{double thetay = atan2(r02, sqrt(r00*r00 + r01*r01));
	double thetax = atan2(-r12, r22);
	double thetaz = atan2(-r01, r00);

	double thetay0 = atan2(-r20, sqrt(r21*r21 + r22*r22));
	double thetax0 = atan2(r21, r22);
	double thetaz0 = atan2(r10, r00);
	
	gtsam::Point3 p(x,-y,z);
    //gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(thetax,thetay,thetaz);  /// experiment
    //gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(2*PI-thetax0,thetay0,2*PI-thetaz0); 
    //gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(thetaz0+PI,thetay0,thetax0); 
    gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(0,PI,0);  

	gtsam::Rot3 rotstd(r00,r01,r02,-r10,-r11,-r12,r20,r21,r22);
	gtsam::Pose3 pose(rot,p);

	std::cout<<"case 1"<<std::endl;
	std::cout<<"rot"<<std::endl;
	rot.print("");
	std::cout<<"rotstd"<<std::endl;
	rotstd.print("");

    cout<<"thetax0: "<< thetax0  << endl;
    cout<<"thetay0: "<< thetay0 << endl;
    cout<<"thetaz0: "<< thetaz0 << endl;

    cout<<"thetax: "<< thetax  << endl;
    cout<<"thetay: "<< thetay << endl;
    cout<<"thetaz: "<< thetaz << endl;

    cout<<"thetax: "<< (thetax > 0 ? thetax : (2*PI + thetax)) * 360 / (2*PI) << endl;
    cout<<"thetay: "<< (thetay > 0 ? thetay : (2*PI + thetay)) * 360 / (2*PI) << endl;
    cout<<"thetaz: "<< (thetaz > 0 ? thetaz : (2*PI + thetaz)) * 360 / (2*PI) << endl;

    cout<<"thetax0: "<< (thetax0 > 0 ? thetax0 : (2*PI + thetax0)) * 360 / (2*PI) << endl;
    cout<<"thetay0: "<< (thetay0 > 0 ? thetay0 : (2*PI + thetay0)) * 360 / (2*PI) << endl;
    cout<<"thetaz0: "<< (thetaz0 > 0 ? thetaz0 : (2*PI + thetaz0)) * 360 / (2*PI) << endl;


	po=pose;
      break;}
   case 2  :
	{float rr00,rr01,rr02,rr10,rr11,rr12,rr20,rr21,rr22;

	rr00=r00;
	rr01=r02;
	rr02=r01;
	rr10=r20;
	rr11=r22;
	rr12=r21;
	rr20=r10;
	rr21=r12;
	rr22=r11;

	double thetay = atan2(rr02, sqrt(rr00*rr00 + rr01*rr01));
	double thetax = atan2(-rr12, rr22);
	double thetaz = atan2(-rr01, rr00);

	double thetay0 = atan2(-rr20, sqrt(rr21*rr21 + rr22*rr22));
	double thetax0 = atan2(rr21, rr22);
	double thetaz0 = atan2(rr10, rr00);

	gtsam::Point3 p(x,z,y);
    gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(thetax,thetay,thetaz);  /// experiment
    //gtsam::Rot3 rot = gtsam::Rot3::RzRyRx(thetax0,thetay0,thetaz0);
	gtsam::Rot3 rotstd(rr00,rr01,rr02,rr10,rr11,rr12,rr20,rr21,rr22);

	std::cout<<"case 2"<<std::endl;
	std::cout<<"rot"<<std::endl;
	rot.print("");
	std::cout<<"rotstd"<<std::endl;
	rotstd.print("");

    cout<<"thetax: "<< (thetax > 0 ? thetax : (2*PI + thetax)) * 360 / (2*PI) << endl;
    cout<<"thetay: "<< (thetay > 0 ? thetay : (2*PI + thetay)) * 360 / (2*PI) << endl;
    cout<<"thetaz: "<< (thetaz > 0 ? thetaz : (2*PI + thetaz)) * 360 / (2*PI) << endl;

    cout<<"thetax0: "<< (thetax0 > 0 ? thetax0 : (2*PI + thetax0)) * 360 / (2*PI) << endl;
    cout<<"thetay0: "<< (thetay0 > 0 ? thetay0 : (2*PI + thetay0)) * 360 / (2*PI) << endl;
    cout<<"thetaz0: "<< (thetaz0 > 0 ? thetaz0 : (2*PI + thetaz0)) * 360 / (2*PI) << endl;

	gtsam::Pose3 pose(rot,p);
	po=pose;
      break;}
 
}

  return po;
}

// Load into isam2 object function
// ----------------------------------------------------------------------------
void loadLoop(gtsam::NonlinearFactorGraph &dataset, gtsam::ISAM2 &isam2){
      // First time step (time from which to process the data file (set this to zero))
  int firstStep = 0;
  // Last time step (time up to which to process the data file (set this to -1))
  int lastStep = -1;

  // Looking for the first observation
  size_t nextObs = 0;
  bool havePreviousPose = false;
  gtsam::Key firstPose;
  // As long as there is an observation to be added to the ISAM problem
  while(nextObs < dataset.size()){

    if(gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr observation = 
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(dataset[nextObs])){
      
      // Get the keys from the observation
      gtsam::Key key1 = observation->key1();
      gtsam::Key key2 = observation->key2();
      
      if((key1 >= firstStep && key1 < key2) || (key2 >= firstStep && key2 < key1)) {
        // This is an odometry starting at firstStep
        firstPose = std::min(key1, key2);
        break;
      }

      if((key2 >= firstStep && key1 < key2) || (key1 >= firstStep && key2 < key1)) {
        // This is an odometry connecting firstStep with a previous pose
        havePreviousPose = true;
        firstPose = std::max(key1, key2);
        break;
      }

    }

    ++ nextObs;

  }

  if (nextObs == dataset.size()){
    std::cout << "The supplied first step is past the end of the dataset." << std::endl;
    exit(1);
  }

  // If we didn't find an odometry linking to a previous pose, create a first pose and a prior
  if(!havePreviousPose){

    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values newVariables;
    //gtsam::Vector v(3);
    //v<<0.01, 0.01, 0.01;
    //gtsam::SharedDiagonal priorNoise = gtsam::noiseModel::Diagonal::Sigmas(v);
    newFactors.push_back(boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(firstPose, gtsam::Pose3(), gtsam::noiseModel::Unit::Create(gtsam::Pose3::dimension)));
    newVariables.insert(firstPose, gtsam::Pose3());
    // Update the ISAM2 problem
    gtsam::ISAM2Result res =  isam2.update(newFactors, newVariables);
    std::cout<<"first factor update "<<std::endl;  
  }
 
  int dsSize=dataset.size();
  for(int d=nextObs;d<dsSize;d++){
         gtsam::NonlinearFactor::shared_ptr obsFactor = dataset[d];
      if(gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr observation = 
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(obsFactor)){
          std::cout<<"obs1 obs2    "<<observation->key1()<<"  "<<observation->key2()<<std::endl;
                gtsam::Values newVariables;
                gtsam::NonlinearFactorGraph newFactors;
                newFactors.push_back(observation);
                
                if(observation->key1() > observation->key2()){
                  if(!isam2.valueExists(observation->key1()) ){                     
                    if(!isam2.valueExists(observation->key2())){
                        std::cout<<"neither exist"<<std::endl;
                    }
                    if(d==nextObs){
                      newVariables.insert(observation->key1(), observation->measured().inverse());
                    }
                    else{
                      gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key2());
                      newVariables.insert(observation->key1(), previousPose * observation->measured().inverse());
                    }
                  }
                }
                else {
                  if(!isam2.valueExists(observation->key2()) ){
                    if(!isam2.valueExists(observation->key1())){
                        std::cout<<"neither exist"<<std::endl;
                    }
                    if (d==nextObs){
                      newVariables.insert(observation->key2(), observation->measured());
                    }
                    else{
                      gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key1());
                      newVariables.insert(observation->key2(), previousPose * observation->measured());
                    }
                  }
                }
                if(d==dsSize-1)
                {
                    gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables,gtsam::FactorIndices(),boost::none,boost::none,boost::none, true);
                    std::cout<<"first kind of update"<<std::endl;
                    std::cout<<"relin-ed = "<< res1.getVariablesRelinearized() <<std::endl;
                }
                else{
                    gtsam::ISAM2Result res1 =  isam2.update(newFactors, newVariables);
                    std::cout<<"second kind of update"<<std::endl;
                    std::cout<<"relin-ed = "<< res1.getVariablesRelinearized() <<std::endl;
                }
      }
      else{
                   std::cout<<"couldnt cast"<<std::endl;
      }
  }
      //gtsam::Values estimate(isam2.calculateEstimate());
}





