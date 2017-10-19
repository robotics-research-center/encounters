#include <string>
#include <vector>

#include "viso_stereo.h"
#include <vector>
#include <fstream>
#include <sstream> 

#include "filewriter.h"
#include "threadcomm.h"

//gtsam includes
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/base/timing.h>


extern comm mythread_comm;
extern writer mywriter;
extern std::vector<Matrix> Tr_global;
extern pthread_mutex_t myMutex;
extern gtsam::NonlinearFactorGraph nfg[2];



#ifndef _HELPER_FINCTIONS_h
#define _HELPER_FINCTIONS_h



// To find number of images in a directory
int listdir(char *dir);


// Class to hold relative poses

class ARsync{

public:
	double thresh=0.3;
	bool detecting[2]={false};
	bool hold[2]={false};
	int framelastseen[2] = {0};
	int framecurrent[2] = {0};
	bool comparing[2]={false};
	double dist[2];
	bool readyToCompare[2]={false};
	bool readyToExchange[2]={false};
	bool exchanged[2]={false};
	int indexfrom[2];
	int indextill[2];
	int edges=0;
	bool finished[2];
};


extern ARsync arsync;

class Tr_relative
{
    public:
      int frame1;
      int frame2;
      Matrix transform;      

};

void master_AR(int numImg,gtsam::ISAM2 &isam2,gtsam::NonlinearFactorGraph &nfg_final,std::vector<bool> &detected, std::vector<gtsam::Pose3> &ARpose);

void compare();

void client_AR(int numImg,gtsam::ISAM2 &isam2,gtsam::NonlinearFactorGraph &nfg_final,std::vector<bool> &detected, std::vector<gtsam::Pose3> &ARpose);

void exchange(int nfgindex, gtsam::NonlinearFactorGraph &nfg_own, gtsam::ISAM2 &isam2, int key);
void remainingadd(int nfgindex, gtsam::NonlinearFactorGraph &nfg_own, gtsam::ISAM2 &isam2, int key);


#endif
