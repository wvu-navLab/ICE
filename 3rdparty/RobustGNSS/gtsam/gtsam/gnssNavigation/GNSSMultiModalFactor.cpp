/**
 *  @file   GNSSMultiModalFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for GNSS factor With mulit-modal uncert. model
 **/

#include <gtsam/gnssNavigation/GNSSMultiModalFactor.h>

using namespace std;
using namespace boost;
using namespace merge;

namespace gtsam {
//***************************************************************************
Vector GNSSMultiModalFactor::evaluateError(const nonBiasStates& q, const phaseBias& g, boost::optional<Matrix&> H1, boost::optional<Matrix&> H2) const {

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        Matrix gnssPartials = Z_1x1;

        if (H1)
        {
                Matrix H_g(2,5);
                H_g.row(0) = h;
                H_g.row(1) = h;
                (*H1) = H_g;
        }
        if (H2)
        {
                Matrix H_b(2,1);
                H_b(0,0) = 0.0;
                H_b(1,0) = 1.0;
                (*H2) = H_b;
        }
        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];

        Eigen::VectorXd res(2);
        res << res_range, res_phase;

        merge::observationModel model= merge::getMixtureComponent(gmm_, res);
        merge::mixtureComponents mixtureComp = model.get<0>();

        Eigen::RowVectorXd mean = mixtureComp.get<3>();
        Eigen::MatrixXd cov = mixtureComp.get<4>();

        noiseModel::Diagonal::shared_ptr newModel = noiseModel::Diagonal::Sigmas((gtsam::Vector(2) << cov(0,0), cov(1,1)).finished());

        SharedDiagonal sharedDiag = boost::dynamic_pointer_cast<
                noiseModel::Diagonal>(newModel);

        *this->noiseModel_ = *sharedDiag;

        return (Vector(2) << res_range - mean(0), res_phase - mean(1)).finished();
}

}  //namespace
