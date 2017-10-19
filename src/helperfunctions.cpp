// To find number of images in a directory

#include <sys/types.h>
#include <dirent.h>
#include "helperfunctions.h"
#include "filewriter.h"
#include "threadcomm.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <stdint.h>
#include <sstream> 

#include <viso_stereo.h>
//#include <png++/png.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pthread.h>

// g2o optimization
#include <g2o/core/sparse_optimizer.h>
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <unistd.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/dataset.h>
#include "g2o/core/factory.h"
#include "g2o/stuff/command_args.h"
#include <stdlib.h>

using namespace std;
using namespace cv;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d)
G2O_USE_TYPE_GROUP(slam3d)

#define PI 3.14159265

writer mywriter;
comm mythread_comm;
std::vector<Matrix> Tr_global;
ARsync arsync;

pthread_mutex_t myMutex;

double valx;
double valz;

int listdir(char *dir) {
    struct dirent *dp;
    DIR *fd;
    int counter = 0;
    //cout << dir<<endl;

    if ((fd = opendir(dir)) == NULL) {
        fprintf(stderr, "listdir: can't open %s\n", dir);
        return -1;
    }
    while ((dp = readdir(fd)) != NULL) {
        if (!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, ".."))
            continue; /* skip self and parent */
        // printf("%s/%s\n", dir, dp->d_name);
        counter++;
        //printf("%s\n", dp->d_name);
    }
    //cout<< counter<<endl;
    closedir(fd);
    return counter;
}

void master_AR(int numImg, gtsam::ISAM2 &isam2, gtsam::NonlinearFactorGraph &nfg_final, std::vector<bool> &detected, std::vector<gtsam::Pose3> &ARpose) { //index into 0 for things
    std::cout << "master!" << std::endl;
    int j = 0;
    arsync.indexfrom[0] = 0;
    arsync.indextill[0] = 0;
    for (int i = 0; i < numImg; i++) {
        std::cout << "master : i: " << i << endl;
        if (i == 0) {
            //init isam2 with prior
            gtsam::NonlinearFactorGraph newFactors;
            gtsam::Values newVariables;
            newFactors.push_back(boost::make_shared<gtsam::PriorFactor<gtsam::Pose3> >(1, gtsam::Pose3(),
                    gtsam::noiseModel::Unit::Create(gtsam::Pose3::dimension)));
            newVariables.insert(1, gtsam::Pose3());
            gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);
            std::cout << "master: added prior" << endl;

        }
        //insert from nfg into nfg final
        //using while loop if next same add that also
        //std::cout<<"master will read nfg"<<std::endl;
        gtsam::NonlinearFactor::shared_ptr factor = nfg[0][j];
        //std::cout<<"master has read nfg"<<std::endl;
        gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr observation =
                boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);

//    

        while (observation->key1() == i + 1) {
            std::cout << "master : while: " << i + 1 << endl;
            //insert----------------------------------------
            nfg_final.push_back(observation);

            gtsam::Values newVariables;
            gtsam::NonlinearFactorGraph newFactors;
            newFactors.push_back(observation);

            if (observation->key1() > observation->key2()) {
                pthread_mutex_lock(&myMutex);
                cout << "master observation->key1() > observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
                pthread_mutex_unlock(&myMutex);
                if (!isam2.valueExists(observation->key1())) {
                    if (!isam2.valueExists(observation->key2())) {
                        throw runtime_error("Problem! None exist");
                    } else {
                        cout << "master inserting key1" << endl;
                        gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key2());
                        newVariables.insert(observation->key1(), previousPose * observation->measured().inverse());
                    }
                }
            }
            else {
                pthread_mutex_lock(&myMutex);
                cout << "master observation->key1() < observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
                pthread_mutex_unlock(&myMutex);
                if (!isam2.valueExists(observation->key2())) {
                    if (!isam2.valueExists(observation->key1())) {
                        throw runtime_error("master Problem! None exist");
                    } else {
                        cout << "master inserting key2" << endl;
                        gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key1());
                        newVariables.insert(observation->key2(), previousPose * observation->measured());
                    }
                }
            }
            if(abs(observation->key1() - observation->key2()) ==1)
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables);
            else
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);
            //	std::cout<<"master isam updated"<<std::endl;
             if(j<nfg[0].size()-1)
            {
                j++;
                factor = nfg[0][j];
                observation = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);   
            }
            else{
                break;
            }
        }
        
               
            while (observation->key1() != i + 2) {
                std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
            std::cout << "master : while!=: " << i + 2 << endl;
            //insert----------------------------------------
            nfg_final.push_back(observation);

            gtsam::Values newVariables;
            gtsam::NonlinearFactorGraph newFactors;
            newFactors.push_back(observation);

            if (observation->key1() > observation->key2()) {
                pthread_mutex_lock(&myMutex);
                cout << "master observation->key1() > observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
                pthread_mutex_unlock(&myMutex);
                if (!isam2.valueExists(observation->key1())) {
                    if (!isam2.valueExists(observation->key2())) {
                        throw runtime_error("Problem! None exist");
                    } else {
                        cout << "master inserting key1" << endl;
                        gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key2());
                        newVariables.insert(observation->key1(), previousPose * observation->measured().inverse());
                    }
                }
            }
            else {
                pthread_mutex_lock(&myMutex);
                cout << "master observation->key1() < observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
                pthread_mutex_unlock(&myMutex);
                if (!isam2.valueExists(observation->key2())) {
                    if (!isam2.valueExists(observation->key1())) {
                        throw runtime_error("master Problem! None exist");
                    } else {
                        cout << "master inserting key2" << endl;
                        gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key1());
                        newVariables.insert(observation->key2(), previousPose * observation->measured());
                    }
                }
            }
            if(abs(observation->key1() - observation->key2()) ==1)
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables);
            else
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);
            //	std::cout<<"master isam updated"<<std::endl;
            if(j<nfg[0].size()-1)
            {
                j++;
                factor = nfg[0][j];
                observation = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);   
            }
            else{
                break;
            }
            std::cout<<"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;

            }
       
        
        arsync.indexfrom[0] = j - 1;
        std::cout<<"master index from = " << arsync.indexfrom[0]<<std::endl;
        arsync.framecurrent[0] = i;

        //if detected
        if (detected[i]) {
            std::cout << "master : detected" << endl;
            //assert(arsync[1].hold==true);
            arsync.hold[0] = true;
            arsync.detecting[0] = true;
            double dist = (ARpose[i].translation()).norm();
            arsync.dist[0] = dist;
            std::cout << "master : dist : " << dist << endl;
            arsync.readyToCompare[0] = true;

            //OTHER DETECTED
            compare();
            if (!arsync.detecting[1]) {
                std::cout << "master : other not detected" << endl;
                arsync.hold[0] = true;
            }

            while (arsync.hold[0]) {
                std::cout << "master : in hold loop" << endl;
                while (!arsync.detecting[1]) {
                    int otherdiff = abs(arsync.framecurrent[1] - arsync.framelastseen[1]);
                    int owndiff = abs(arsync.framecurrent[0] - arsync.framelastseen[0]);
                    int prevothercurrent=arsync.framecurrent[1];
                    int othercurrent=arsync.framecurrent[1];
                    while (!arsync.detecting[1] && (otherdiff > 200) && (owndiff > 70)) {
                        othercurrent=arsync.framecurrent[1];
                        if(abs(prevothercurrent-othercurrent)>700) {
                            std::cout << "master : CONDITION A" << endl;
                            arsync.hold[0] = false;
                            break;   
                        }
                    }
                    if (!arsync.detecting[1] && (otherdiff > 30) && (owndiff < 10)) {
                        std::cout << "master : CONDITION B" << endl;
                        arsync.hold[0] = false;
                        break;
                    }

                    sleep(1);
                }

                if (!arsync.detecting[1]) {
                    break;
                }
                compare();


                //if conditions met -------------------------------------------------------------------------
                if (arsync.readyToExchange[0] && arsync.readyToExchange[1]) {
                    //add AR
                    arsync.edges++;
                    gtsam::Matrix M(6, 6);
                    M << 1000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 1000;
                    gtsam::SharedNoiseModel model;
                    model = gtsam::noiseModel::Gaussian::Information(M, true);
                    gtsam::NonlinearFactorGraph newFactors;
                    gtsam::Values newVariables2;

                    gtsam::NonlinearFactor::shared_ptr factor(
                            new gtsam::BetweenFactor<gtsam::Pose3>(i + 1, 10000 + 1 + arsync.framecurrent[1], ARpose[i], model)); //model!!
                    nfg_final.push_back(factor);
                    factor->print("");
                    newFactors.push_back(factor);
                    gtsam::Pose3 pose = isam2.calculateEstimate<gtsam::Pose3>(i + 1);
                    pose.print("");
                    std::cout << "key " << 10000 + 1 + arsync.framecurrent[1] << std::endl;
                    if (!isam2.valueExists(10000 + 1 + arsync.framecurrent[1])) {
                        newVariables2.insert(10000 + 1 + arsync.framecurrent[1], pose * ARpose[i]);
                    }
                    std::cout << "master: going to add AR constraint" << std::endl;
                    gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables2, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);

                    std::cout << "master: added AR constraint" << std::endl;
                    exchange(0, nfg_final, isam2, 10000 + 1 + arsync.framecurrent[1]);
                    std::cout << "master: exchange call over" << std::endl;

                    arsync.exchanged[0] = true;
                    //add buffer time here?
                    while (!arsync.exchanged[1]) {
                    }
                    if (arsync.exchanged[1]) {
                        std::cout << "master: other also exchanged" << std::endl;

                        arsync.indextill[0] = j;
                        arsync.readyToExchange[0] = false;
                        arsync.exchanged[0] = false;
                        break;
                    }
                }
                //------------------------------------------------------------------------------------------


            }

            arsync.hold[0] = false;
            arsync.detecting[0] = false;
            arsync.readyToCompare[0] = false;
            arsync.framelastseen[0] = i;

        }

            //OTHER NOT DETECTING

            //not detected

        else {
            std::cout << "master : not detected " << endl;
            arsync.dist[0] = 99;
            arsync.detecting[0] = false;
        }

        arsync.detecting[0] = false;

    }
    arsync.finished[0]=true;
    while(!arsync.finished[1]){}
    remainingadd(0, nfg_final, isam2, 0); //key put in

    string outputf = "master-nfg.g2o";
    string outputfi = "master-isamnfg.g2o";
    gtsam::Values estimate(isam2.calculateEstimate());
    gtsam::writeG2o(nfg_final, gtsam::Values(), outputf);
    gtsam::writeG2o(nfg_final, estimate, outputfi);
    


}

void compare() { // will only be called by master

    std::cout << "in compare func" << endl;
    if (arsync.detecting[1]) {
        if (!arsync.hold[1]) {
            std::cout << "NOT ON HOLD" << std::endl;
            while (!arsync.hold[1]) {
            }
            std::cout << "out of compare on hold check in while loop" << std::endl;

            //throw runtime_error("NOT ON HOLD");
        }
        while (!arsync.readyToCompare[0] || !arsync.readyToCompare[1]) {
            if (!arsync.detecting[1]) {
                std::cout << "THIS HAPPENS TOO" << std::endl;
                throw runtime_error("THIS HAPPENS TOO");
            }

        }
        //add check if truly ready to compare or just has broken out of loop
        //comparing
        if (arsync.readyToCompare[0] && arsync.readyToCompare[1]) {
            std::cout << "both ready to compare" << endl;
            arsync.comparing[0] = true;
            if (abs(arsync.dist[1] - arsync.dist[0]) < arsync.thresh) {
                arsync.readyToExchange[0] = true;
                arsync.readyToExchange[1] = true;
                arsync.exchanged[0] = false;
                arsync.exchanged[1] = false;
                std::cout << "both ready to exchange" << endl;
            } else {
                if (arsync.dist[1] < arsync.dist[0]) {
                    pthread_mutex_lock(&myMutex);
                    arsync.hold[1] = true; //mutex
                    arsync.hold[0] = false;
                    pthread_mutex_unlock(&myMutex);
                    std::cout << "0(master) asked to move closer" << endl;


                } else {
                    pthread_mutex_lock(&myMutex);
                    arsync.hold[0] = true; //mutex
                    arsync.hold[1] = false;
                    pthread_mutex_unlock(&myMutex);
                    std::cout << "1(client) asked to move closer" << endl;


                }
            }

        }


        arsync.comparing[0] = false;
    }

    std::cout << "getting out of compare func" << endl;

}

void client_AR(int numImg, gtsam::ISAM2 &isam2, gtsam::NonlinearFactorGraph &nfg_final, std::vector<bool> &detected, std::vector<gtsam::Pose3> &ARpose) {

    string outputf = "client-nfg.g2o";
    string outputfi = "client-isamnfg.g2o";
    std::cout << "							client!" << std::endl;

    int j = 0;
    arsync.indexfrom[1] = 0;
    arsync.indextill[1] = 0;

    for (int i = 0; i < numImg; i++) {
        if(i==1500){
            gtsam::Values estimate(isam2.calculateEstimate());
            gtsam::writeG2o(nfg_final, gtsam::Values(), outputf);
            gtsam::writeG2o(nfg_final, estimate, outputfi);
        }
        std::cout << "							client : i: " << i << endl;
        if (i == 0) {
            //init isam2 with prior
            gtsam::NonlinearFactorGraph newFactors;
            gtsam::Values newVariables;
            newFactors.push_back(boost::make_shared<gtsam::PriorFactor<gtsam::Pose3> >(10001, gtsam::Pose3(),
                    gtsam::noiseModel::Unit::Create(gtsam::Pose3::dimension)));
            newVariables.insert(10001, gtsam::Pose3()); //!!
            gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);
            std::cout << "							client : added prior " << endl;

        }
        //insert from nfg into nfg final
        //using while loop if next same add that also
        //std::cout<<"									client will read nfg"<<std::endl;
        gtsam::NonlinearFactor::shared_ptr factor = nfg[1][j];
        //std::cout<<"									client has read nfg"<<std::endl;
        gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr observation =
                boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);

        while (observation->key1() == 10000 + i + 1) {
            std::cout << "							client : while : " << 10000 + i + 1 << endl;
            //insert----------------------------------------
            nfg_final.push_back(observation);
            gtsam::Values newVariables;
            gtsam::NonlinearFactorGraph newFactors;
            newFactors.push_back(observation);

            if (observation->key1() > observation->key2()) {
                pthread_mutex_lock(&myMutex);
                cout << "							client observation->key1() > observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
                pthread_mutex_unlock(&myMutex);
                if (!isam2.valueExists(observation->key1())) {
                    if (!isam2.valueExists(observation->key2())) {
                        throw runtime_error("							Problem! None exist");
                    } else {
                        cout << "							client inserting key1" << endl;
                        gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key2());
                        newVariables.insert(observation->key1(), previousPose * observation->measured().inverse());
                    }
                }
            }
            else {
                pthread_mutex_lock(&myMutex);
                cout << "							client observation->key1() < observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
                pthread_mutex_unlock(&myMutex);
                if (!isam2.valueExists(observation->key2())) {
                    if (!isam2.valueExists(observation->key1())) {
                        throw runtime_error("Problem! None exist");
                    } else {
                        cout << "							client inserting key2" << endl;
                        gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key1());
                        newVariables.insert(observation->key2(), previousPose * observation->measured());
                        std::cout << "							client inserted key2" << std::endl;
                    }
                }
            }
            //------------------------------------------------------
            if(abs(observation->key1() - observation->key2()) ==1)
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables);
            else
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);

            if(j<nfg[1].size()-1)
            {
                j++;
                factor = nfg[1][j];
                observation = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);   
            }
            else{
                break;
            }

        }
                    
            while (observation->key1() != 10000 + i + 2) {
            std::cout<<"							<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;

            std::cout << "							master : while!=: " << i + 1 << endl;
            //insert----------------------------------------
            nfg_final.push_back(observation);

            gtsam::Values newVariables;
            gtsam::NonlinearFactorGraph newFactors;
            newFactors.push_back(observation);

            if (observation->key1() > observation->key2()) {
                pthread_mutex_lock(&myMutex);
                cout << "							client observation->key1() > observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
                pthread_mutex_unlock(&myMutex);
                if (!isam2.valueExists(observation->key1())) {
                    if (!isam2.valueExists(observation->key2())) {
                        throw runtime_error("Problem! None exist");
                    } else {
                        cout << "							client inserting key1" << endl;
                        gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key2());
                        newVariables.insert(observation->key1(), previousPose * observation->measured().inverse());
                    }
                }
            }
            else {
                pthread_mutex_lock(&myMutex);
                cout << "							client observation->key1() < observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
                pthread_mutex_unlock(&myMutex);
                if (!isam2.valueExists(observation->key2())) {
                    if (!isam2.valueExists(observation->key1())) {
                        throw runtime_error("master Problem! None exist");
                    } else {
                        cout << "							client inserting key2" << endl;
                        gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key1());
                        newVariables.insert(observation->key2(), previousPose * observation->measured());
                    }
                }
            }
            if(abs(observation->key1() - observation->key2()) ==1)
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables);
            else
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);
            //	std::cout<<"master isam updated"<<std::endl;
            if(j<nfg[1].size()-1)
            {
                j++;
                factor = nfg[1][j];
                observation = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);   
            }
            else{
                break;
            }   
            std::cout<<"							<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;

        }
        
        arsync.indexfrom[1] = j - 1;
        std::cout<<"							client index from = " << arsync.indexfrom[1]<<std::endl;
        arsync.framecurrent[1] = i;

        //if detected
        if (detected[i]) {
            std::cout << "							client : detected " << endl;
            //assert(arsync[1].hold==true);
            arsync.hold[1] = true;
            arsync.detecting[1] = true;
            double dist = (ARpose[i].translation()).norm();
            std::cout << "							client : dist : " << dist << endl;
            arsync.dist[1] = dist;
            arsync.readyToCompare[1] = true;







            //check this whole situation//////////////////////////////////////////////////////////////
            while (arsync.hold[1]) {
                int otherdiff = abs(arsync.framecurrent[0] - arsync.framelastseen[0]);
                int owndiff = abs(arsync.framecurrent[1] - arsync.framelastseen[1]);
                
                int prevothercurrent=arsync.framecurrent[0];
                int othercurrent=arsync.framecurrent[0];
                bool breakout=false;
                while (!arsync.detecting[0] && (otherdiff > 200) && (owndiff > 100)) {
                    othercurrent=arsync.framecurrent[0];
                    if (abs(prevothercurrent - othercurrent) > 700) {
                        breakout = true;
                        std::cout << "								client : CONDITION C " << endl;
                        arsync.hold[1] = false;
                        break;                        
                    }

                } 
                if (!arsync.detecting[0] && (otherdiff > 20) && (owndiff < 10)) {
                    std::cout << "							client : CONDITION D " << endl; //add more here
                    arsync.hold[1] = false;
                    break;
                }

                if(breakout){
                    break;
                }
                if (!arsync.detecting[0]) {
                    sleep(1);
                }

                //if condition met -------------------------------
                if (arsync.readyToExchange[0] && arsync.readyToExchange[1]) {
                    std::cout << "							client : both ready " << endl;
                    //add AR

                    gtsam::NonlinearFactorGraph newFactors;
                    gtsam::Values newVariables;
                    gtsam::Matrix M(6, 6);
                    M << 1000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 1000, 0, 0, 0, 0, 0, 0, 1000;
                    gtsam::SharedNoiseModel model;
                    model = gtsam::noiseModel::Gaussian::Information(M, true);

                    gtsam::NonlinearFactor::shared_ptr factor(
                            new gtsam::BetweenFactor<gtsam::Pose3>(10000 + 1 + i, 1 + arsync.framecurrent[0], ARpose[i], model)); //model!!
                    nfg_final.push_back(factor);
                    newFactors.push_back(factor);
                    gtsam::Pose3 pose = isam2.calculateEstimate<gtsam::Pose3>(10000 + 1 + i);
                    if (!isam2.valueExists(1 + arsync.framecurrent[0])) {
                        newVariables.insert(1 + arsync.framecurrent[0], pose * ARpose[i]);
                    }
                    gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);
                    std::cout << "							client: AR constraint added" << std::endl;
                    while (!arsync.exchanged[0]) {
                        sleep(1);
                        std::cout << "									client : waiting to master to finish exchange " << endl;
                    }
                    if (arsync.exchanged[0]) {
                        std::cout << "							client: other's exchange call over" << std::endl;

                        exchange(1, nfg_final, isam2, 1 + arsync.framecurrent[0]);
                        std::cout << "									client : exchange call over" << endl;

                        arsync.indextill[1] = j;
                        arsync.readyToExchange[1] = false;
                    }

                    //set some flag to say comparing of 0 done

                    arsync.exchanged[1] = true;
                    //add buffer time here?
                    arsync.readyToExchange[1] = false;
                    break;

                }
                //-------------------------------------------------


            }

            arsync.hold[1] = false;
            arsync.detecting[1] = false;
            arsync.readyToCompare[1] = false;
            arsync.framelastseen[1] = i;

        }
            //not detected

        else {
            std::cout << "							client : not detected " << endl;
            arsync.dist[1] = 99;
            arsync.detecting[1] = false;
        }

        arsync.detecting[1] = false;

    }
    arsync.finished[1]=true;
    while(!arsync.finished[0]){}
    remainingadd(1, nfg_final, isam2, 0); //key put in


    gtsam::Values estimate(isam2.calculateEstimate());
    gtsam::writeG2o(nfg_final, gtsam::Values(), outputf);
    gtsam::writeG2o(nfg_final, estimate, outputfi);

}

//add AR constraints
//proofread exchange function

void exchange(int nfgindex, gtsam::NonlinearFactorGraph &nfg_own, gtsam::ISAM2 &isam2, int key) {
    cout << endl;
    cout << endl;
    cout << endl;
    std::cout << "-----------------exchange : " << nfgindex << endl;
    std::cout << "-----------------exchange : key : " << key << endl;
    //while loop
    //int nfgsize=nfg[1-nfgindex].size();
    int marker = arsync.indexfrom[1 - nfgindex];
    std::cout << "marker : " << marker << std::endl;
    gtsam::NonlinearFactor::shared_ptr factor = nfg[1 - nfgindex][marker];
    gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr observation =
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);
    int added = 0;
    std::cout << "observation->key1() : " << observation->key1() << std::endl;
    while (observation->key1() == key) {
        std::cout<< "observation->key1() == key which is " << key <<std::endl;
        added++;
        //insert----------------------------------------
        nfg_own.push_back(observation);
        gtsam::Values newVariables;
        gtsam::NonlinearFactorGraph newFactors;
        newFactors.push_back(observation);
        cout << " observation->key1(), observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
        if (observation->key1() > observation->key2()) {

            if (!isam2.valueExists(observation->key2())) {
                if (!isam2.valueExists(observation->key1())) {
                    throw runtime_error("Problem! None exist");
                } else {
                    cout << " inserting key1" << endl;
                    gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key1());
                    newVariables.insert(observation->key2(), previousPose * observation->measured());
                }
            }
        }
        else {
            cout << " observation->key1() < observation->key2()" << endl;
            if (!isam2.valueExists(observation->key2())) {
                if (!isam2.valueExists(observation->key1())) {
                    throw runtime_error("Problem! None exist");
                } else {
                    cout << " inserting key2" << endl;
                    gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key1());
                    newVariables.insert(observation->key2(), previousPose * observation->measured());
                }
            }
        }
        //------------------------------------------------------
            if(abs(observation->key1() - observation->key2()) ==1)
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables);
            else
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);

        marker--;
        factor = nfg[1 - nfgindex][marker];
        observation = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);
    }




    int obs = arsync.indexfrom[1 - nfgindex];
    std::cout << "marker added obs : " << marker << " " << added << " " << obs << endl;
    assert(obs - added == marker);
    obs = marker;
    int till = arsync.indextill[1 - nfgindex];
    std::cout << "till : " << till << std::endl;
    for (; obs >= till; obs--) {
        gtsam::NonlinearFactor::shared_ptr factor = nfg[1 - nfgindex][obs];
        gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr observation =
                boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);

        nfg_own.push_back(observation);
        gtsam::Values newVariables;
        gtsam::NonlinearFactorGraph newFactors;
        newFactors.push_back(observation);
        cout << " observation->key1(), observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
        if (observation->key1() > observation->key2()) {
            throw runtime_error("should not happen");
            if (!isam2.valueExists(observation->key2())) {
                if (!isam2.valueExists(observation->key1())) {
                    throw runtime_error("Problem! None exist");
                } else {
                    cout << " inserting key2" << endl;
                    gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key1());
                    newVariables.insert(observation->key2(), previousPose * observation->measured());
                }
            }
        } else {
            if (!isam2.valueExists(observation->key1())) {
                if (!isam2.valueExists(observation->key2())) {
                    if (abs(observation->key2() - observation->key1()) == 5) {
                        continue;
                    } else {
                        throw runtime_error("Problem! None exist");
                    }
                } else {
                    cout << " inserting key1" << endl;
                    gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key2());
                    newVariables.insert(observation->key1(), previousPose * observation->measured().inverse());
                }
            }
            else if(!isam2.valueExists(observation->key2())){
                if (abs(observation->key2() - observation->key1()) == 5) {
                    continue;
                }
            }
        }


            if(abs(observation->key1() - observation->key2()) ==1)
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables);
            else
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);


    }


    cout << "------------------------getting out of exchange " << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;


}

void remainingadd(int nfgindex, gtsam::NonlinearFactorGraph &nfg_own, gtsam::ISAM2 &isam2, int key){
    if(nfgindex==0)
        cout<<"master remainingadd"<<endl;
    else
        cout<<"client remainingadd"<<endl;
                
    //add remaining factors to this persons nfg_final and isam
    int nfgsize=nfg[1-nfgindex].size();
    int checkfrom = arsync.indexfrom[1 - nfgindex];
    int till = arsync.indextill[1 - nfgindex];
    int obs=nfgsize-1;
    std::cout<<"checkfrom "<< checkfrom << std::endl;
    std::cout<<"obs "<< obs << std::endl;

    assert(abs(checkfrom-obs)<=1);

    std::cout << "obs : " << obs << endl;
    std::cout << "till : " << till << std::endl;
    
    for (; till<=obs; till++) {
        gtsam::NonlinearFactor::shared_ptr factor = nfg[1 - nfgindex][till];
        gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr observation =
                boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factor);

        nfg_own.push_back(observation);
        gtsam::Values newVariables;
        gtsam::NonlinearFactorGraph newFactors;
        newFactors.push_back(observation);
        cout << " observation->key1(), observation->key2() : " << observation->key1() << " " << observation->key2() << endl;
        if (observation->key1() > observation->key2()) {
            if (!isam2.valueExists(observation->key2())) {
                if (!isam2.valueExists(observation->key1())) {
                    throw runtime_error("Problem! None exist");
                } else {
                    cout << " inserting key1" << endl;
                    gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key2());
                    newVariables.insert(observation->key1(), previousPose * observation->measured().inverse());
                }
            }
        } else {
            if (!isam2.valueExists(observation->key2())) {
                if (!isam2.valueExists(observation->key1())) {
                        throw runtime_error("Problem! None exist");
                }
                else {
                    cout << " inserting key2" << endl;
                    gtsam::Pose3 previousPose = isam2.calculateEstimate<gtsam::Pose3>(observation->key1());
                    newVariables.insert(observation->key2(), previousPose * observation->measured());
                }
            }
        }


            if(abs(observation->key1() - observation->key2()) ==1)
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables);
            else
                gtsam::ISAM2Result res1 = isam2.update(newFactors, newVariables, gtsam::FactorIndices(), boost::none, boost::none, boost::none, true);


    }


    cout << "------------------------getting out of remaining " << endl;
    cout << endl;
    cout << endl;
    cout << endl;
    cout << endl;
}