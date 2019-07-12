/*
 * @file pppBayesTree.cpp
 * @brief Iterative GPS Range/Phase Estimator with collected data
 * @author Ryan Watson & Jason Gross
 */

// GTSAM related includes.
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/gnssNavigation/GnssData.h>
#include <gtsam/gnssNavigation/GnssTools.h>
#include <gtsam/gnssNavigation/GNSSFactor.h>
#include <gtsam/gnssNavigation/PhaseFactor.h>
#include <gtsam/gnssNavigation/nonBiasStates.h>
#include <gtsam/configReader/ConfDataReader.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/gnssNavigation/PseudorangeFactor.h>
#include <gtsam/gnssNavigation/GNSSMultiModalFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>



// LibCluster
#include <libcluster/merge.h>
#include <libcluster/probutils.h>
#include <libcluster/libcluster.h>
#include <libcluster/distributions.h>

// BOOST
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/serialization/export.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>


#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>

using namespace std;
using namespace gtsam;
using namespace gpstk;
using namespace boost;
using namespace merge;
using namespace std::chrono;
using namespace libcluster;
using namespace distributions;
namespace NM = gtsam::noiseModel;
namespace po = boost::program_options;
typedef noiseModel::Diagonal diagNoise;

// Intel Threading Building Block
#ifdef GTSAM_USE_TBB
  #include <tbb/tbb.h>
  #undef max // TBB seems to include windows.h and we don't want these macros
  #undef min
#endif

using symbol_shorthand::X; // nonBiasStates ( dx, dy, dz, trop, cb )
using symbol_shorthand::G;   // bias states ( Phase Biases )

int main(int argc, char* argv[])
{
        // define std out print color
        vector<int> prn_vec;
        vector<int> factor_count_vec;
        vector<rnxData> data;
        const string red("\033[0;31m");
        const string green("\033[0;32m");
        string confFile, gnssFile, station;
        double xn, yn, zn, range, phase, rho, gnssTime;
        int startKey(0), currKey, startEpoch(0), svn;
        int nThreads(-1), phase_break, break_count(0), nextKey, factor_count(-1), res_count(-1);
        bool printECEF, printENU, printAmb, first_ob(true);
        Eigen::MatrixXd residuals;
        vector<mixtureComponents> globalMixtureModel;


        cout.precision(12);

        po::options_description desc("Available options");
        desc.add_options()
                ("help,h", "Print help message")
                ("confFile,c", po::value<string>(&confFile)->default_value(""),
                "Input config file" );

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        po::notify(vm);

        ConfDataReader confReader;
        confReader.open(confFile);

        if (confFile.empty() ) {
                cout << red << "\n\n Currently, you need to provide a conf file \n"
                     << "\n\n"  << green << desc << endl;
        }

        while ( (station = confReader.getEachSection()) != "" )
        {
                xn = confReader.fetchListValueAsDouble("nominalECEF",station);
                yn = confReader.fetchListValueAsDouble("nominalECEF",station);
                zn = confReader.fetchListValueAsDouble("nominalECEF",station);
                printENU = confReader.getValueAsBoolean("printENU", station);
                printAmb = confReader.getValueAsBoolean("printAmb", station);
                printECEF = confReader.getValueAsBoolean("printECEF", station);
                gnssFile = confReader("dataFile", station);
        }

        Point3 nomXYZ(xn, yn, zn);
        Point3 prop_xyz = nomXYZ;

        try { data = readGNSS(gnssFile); }
        catch(std::exception& e)
        {
                cout << red << "\n\n Cannot read GNSS data file " << endl;
                exit(1);
        }

        #ifdef GTSAM_USE_TBB
        std::auto_ptr<tbb::task_scheduler_init> init;
        if(nThreads > 0) {
                init.reset(new tbb::task_scheduler_init(nThreads));
        }
        else
                cout << green << " \n\n Using threads for all processors" << endl;
        #else
        if(nThreads > 0) {
                cout << red <<" \n\n GTSAM is not compiled with TBB, so threading is"
                     << " disabled and the --threads option cannot be used."
                     << endl;
                exit(1);
        }
        #endif

        ISAM2DoglegParams doglegParams;
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 100;
        ISAM2 isam(parameters);

        double output_time = 0.0;
        double rangeWeight = pow(2.5,2);
        double phaseWeight = pow(0.25,2);

        ifstream file(gnssFile.c_str());
        string value;

        nonBiasStates prior_nonBias = (gtsam::Vector(5) << 0.0, 0.0, 0.0, 0.0, 0.0).finished();

        phaseBias bias_state(Z_1x1);
        gnssStateVector phase_arc(Z_34x1);
        gnssStateVector bias_counter(Z_34x1);
        for (int i=1; i<34; i++) {bias_counter(i) = bias_counter(i-1) + 10000; }

        nonBiasStates initEst(Z_5x1);
        nonBiasStates between_nonBias_State(Z_5x1);

        Values initial_values;
        Values result;

        noiseModel::Diagonal::shared_ptr nonBias_InitNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 3.0, 3.0, 3.0, 1e3, 1e-1).finished());

        noiseModel::Diagonal::shared_ptr nonBias_ProcessNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(5) << 1.0, 1.0, 1.0, 10, 1e-3).finished());

        noiseModel::Diagonal::shared_ptr initNoise = noiseModel::Diagonal::Sigmas((gtsam::Vector(1) << 1).finished());


        NonlinearFactorGraph *graph = new NonlinearFactorGraph();

        residuals.setZero(1000,2);

        // init. mixture model.
        // Init this from file later
        Eigen::RowVectorXd m(2);
        m << 0.0, 0.0;

        Eigen::MatrixXd c(2,2);
        c<< rangeWeight, 0.0, 0.0, phaseWeight;

        globalMixtureModel.push_back(boost::make_tuple(0, 0, 0.0, m, c));

        for(unsigned int i = startEpoch; i < data.size(); i++ ) {

                // Get the current epoch's observables
                gnssTime = get<0>(data[i]);
                currKey = get<1>(data[i]);
                if (first_ob) {
                        startKey = currKey; first_ob=false;
                        graph->add(PriorFactor<nonBiasStates>(X(currKey), initEst,  nonBias_InitNoise));
                        ++factor_count;
                        initial_values.insert(X(currKey), initEst);
                }
                nextKey = get<1>(data[i+1]);
                svn = get<2>(data[i]);
                Point3 satXYZ = get<3>(data[i]);
                rho = get<4>(data[i]);
                range = get<5>(data[i]);
                phase = get<6>(data[i]);
                phase_break = get<7>(data[i]);
                gtsam::Vector2 obs;
                obs << range-rho, phase-rho;


                if (phase_arc[svn]!=phase_break)
                {
                        bias_state[0] = phase - range;
                        if (currKey > startKey) { bias_counter[svn] = bias_counter[svn] +1; }
                        graph->add(PriorFactor<phaseBias>(G(bias_counter[svn]), bias_state,  initNoise));
                        ++factor_count;
                        initial_values.insert(G(bias_counter[svn]), bias_state);
                        phase_arc[svn] = phase_break;
                }

                graph->add(boost::make_shared<GNSSMultiModalFactor>(X(currKey), G(bias_counter[svn]), obs, satXYZ, nomXYZ, diagNoise::Variances( (gtsam::Vector(2) << rangeWeight, phaseWeight).finished() ), globalMixtureModel));

                prn_vec.push_back(svn);
                factor_count_vec.push_back(++factor_count);

                if (currKey != nextKey) {
                        if (currKey > startKey ) {
                                graph->add(BetweenFactor<nonBiasStates>(X(currKey), X(currKey-1), between_nonBias_State, nonBias_ProcessNoise));
                                ++factor_count;
                        }
                        isam.update(*graph, initial_values);
                        result = isam.calculateEstimate();

                        prior_nonBias = result.at<nonBiasStates>(X(currKey));
                        Point3 delta_xyz = (gtsam::Vector(3) << prior_nonBias.x(), prior_nonBias.y(), prior_nonBias.z()).finished();
                        prop_xyz = nomXYZ - delta_xyz;

                        if (printECEF) {
                                cout << "xyz " << gnssTime << " " << prop_xyz.x() << " " << prop_xyz.y() << " " << prop_xyz.z() << endl;
                        }

                        if (printENU) {
                                Point3 enu = xyz2enu(prop_xyz, nomXYZ);
                                cout << "enu " << gnssTime << " " << enu.x() << " " << enu.y() << " " << enu.z() << endl;
                        }

                        if (printAmb) {
                                cout << "gps " << " " << gnssTime << " ";
                                for (int k=0; k<prn_vec.size(); k++) {
                                        cout << result.at<phaseBias>(G(bias_counter[prn_vec[k]])) << " ";
                                }
                                cout << endl;
                        }

                        output_time = output_time +1;

                        // Get residuals from graph
                        for (int i = 0; i<factor_count_vec.size(); i++) {
                                ++res_count;
                                if (res_count > 999 )
                                {
                                        residuals.conservativeResize(residuals.rows()+1, residuals.cols());

                                        residuals.row(residuals.rows()-1) = graph->at(factor_count_vec[i])->residual(result).transpose();
                                }
                                else
                                        residuals.block(res_count,0,1,2) << graph->at(factor_count_vec[i])->residual(result).transpose();
                        }
                        factor_count_vec.clear();
                        factor_count = -1;


                        if (res_count >= 1000)
                        {
                                StickBreak weights;
                                vector<GaussWish> clusters;
                                Eigen::MatrixXd qZ;

                                learnVDP(residuals, qZ, weights, clusters);

                                globalMixtureModel = mergeMixtureModel(residuals, qZ, globalMixtureModel, clusters, weights, 0.05, 10);

                                // cout << "\n\n\n\n\n\n" << endl;
                                // cout << "----------------- Merged MODEL ----------------" << endl;
                                // for (int i=0; i<globalMixtureModel.size(); i++)
                                // {
                                //         mixtureComponents mc = globalMixtureModel[i];
                                //         auto cov = mc.get<4>();
                                //         cout << mc.get<0>() << " " << mc.get<1>() << " "  <<  mc.get<2>() << "    " << mc.get<3>() <<"     "<< cov(0,0) << " " << cov(0,1) << " " << cov(1,1) <<"     "<<"\n\n" << endl;
                                // }
                                //

                                residuals.setZero(1000,2);
                                res_count = -1;
                        }

                        graph->resize(0);
                        initial_values.clear();
                        prn_vec.clear();
                        initial_values.insert(X(nextKey), prior_nonBias);
                }

        }
        isam.saveGraph("gnss.tree");
        return 0;
}
