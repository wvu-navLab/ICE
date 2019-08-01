/**
 *  @file   GNSSMultiModalFactor.cpp
 *  @author Ryan Watson
 *  @brief  Implementation file for GNSS factor With mulit-modal uncert. model
 **/

#include <gtsam/gnssNavigation/GNSSMultiModalFactor.h>

using namespace std;
using namespace boost;
using namespace merge;

namespace gtsam {
//***************************************************************************
Vector GNSSMultiModalFactor::whitenedError(const gtsam::Values& x,
                                           boost::optional<std::vector<Matrix>&> H) const {


        const nonBiasStates& q = x.at<nonBiasStates>(k1_);
        const phaseBias& g = x.at<phaseBias>(k2_);

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        Matrix gnssPartials = Z_1x1;

        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];

        Eigen::VectorXd res(2);
        res << res_range, res_phase;



        double e, eMin;
        eMin = 1e100;
        int ind(0);
        gtsam::Matrix cov_min;
        Eigen::RowVectorXd mean_min;
        for (int i=0; i<gmm_.size(); i++)
        {
                merge::mixtureComponents mixtureComp = gmm_[i];
                Eigen::MatrixXd cov = mixtureComp.get<4>();
                gtsam::Matrix c(2,2);
                c << cov(0,0), cov(0,1), cov(1,0), cov(1,1);
                e = ((gtsam::noiseModel::Gaussian::Covariance(c))->whiten(res)).squaredNorm();

                if (e < eMin)
                {
                        ind = i;
                        eMin = e;
                        cov_min = c;
                        mean_min = mixtureComp.get<3>();
                }

        }

        Eigen::VectorXd res_2(2);
        res_2 << res_range - mean_min(0), res_phase - mean_min(1);

        if (H) {
                Matrix H_g(2,5);
                H_g.row(0) = h;
                H_g.row(1) = h;

                Matrix H_b(2,1);
                H_b(0,0) = 0.0;
                H_b(1,0) = 1.0;

                (*H)[0].resize(H_g.rows(), H_g.cols());
                (*H)[1].resize(H_b.rows(), H_b.cols());

                (*H)[0] = H_g;
                (*H)[1] = H_b;
        }

        return (gtsam::noiseModel::Gaussian::Covariance(cov_min))->whiten(res_2);
}

Vector GNSSMultiModalFactor::unwhitenedError(const gtsam::Values& x,
                                             boost::optional<std::vector<Matrix>&> H) const {

        const nonBiasStates& q = x.at<nonBiasStates>(k1_);
        const phaseBias& g = x.at<phaseBias>(k2_);

        Vector h = obsMap(satXYZ_, nomXYZ_, 1);
        Matrix gnssPartials = Z_1x1;

        double res_range = (h.transpose() * q) - measured_[0];
        double res_phase = (h.transpose() * q) + g[0] - measured_[1];

        Eigen::VectorXd res(2);
        res << res_range, res_phase;

        merge::observationModel model= merge::getMixtureComponent(gmm_, res);
        merge::mixtureComponents mixtureComp = model.get<0>();

        Eigen::RowVectorXd mean = mixtureComp.get<3>();
        Eigen::VectorXd res_2(2);
        res_2 << res_range - mean(0), res_phase - mean(1);

        if (H) {

                Matrix H_g(2,5);
                H_g.row(0) = h;
                H_g.row(1) = h;

                Matrix H_b(2,1);
                H_b(0,0) = 0.0;
                H_b(1,0) = 1.0;

                (*H)[0].resize(H_g.rows(), H_g.cols());
                (*H)[1].resize(H_b.rows(), H_b.cols());

                (*H)[0] = H_g;
                (*H)[1] = H_b;
        }


        return res_2;
}

}  //namespace
