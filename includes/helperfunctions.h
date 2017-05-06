
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
extern gtsam::Values initial;
//extern std::ofstream unopt; //needed?
extern gtsam::Values initialunopt;
extern gtsam::Pose3 refPose; 
extern gtsam::Pose3 currentPose; 
extern int latestkey;

#ifndef _HELPER_FINCTIONS_h
#define _HELPER_FINCTIONS_h

// extern bool dloop_wait_flag;
// extern bool viso_wait_flag;
// extern bool loop_write_done;
//extern ofstream myfile1;

// To find number of images in a directory
int listdir(char *dir);

// To find Visual Odometry using libviso2
void my_libviso2(std::string dir, int numImg,gtsam::ISAM2 &isam2,gtsam::NonlinearFactorGraph &nfg);

// Class to hold relative poses

class Tr_relative
{
    public:
      int frame1;
      int frame2;
      Matrix transform;      

};

// To find relative transformations between loop closing frames
void my_libviso2_relative(std::vector<Tr_relative> &relative, std::vector<int> index1, std::vector<int> index2, std::string dir);

bool my_libviso2_relative(Matrix &Tr_final, int index1, int index2, std::string dir,std::string dir2);

std::string my_for_g2o_edge(int id1, int id2, Matrix transform,gtsam::ISAM2 &isam2,bool relin,gtsam::NonlinearFactorGraph &nfg,std::string prefix);


#endif
