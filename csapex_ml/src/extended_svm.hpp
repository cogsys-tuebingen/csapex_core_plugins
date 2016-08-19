#ifndef EXTENDED_SVM_HPP
#define EXTENDED_SVM_HPP

#include <memory>
#include <opencv2/opencv.hpp>


namespace csapex {
struct ExtendedSVM : public cv::SVM {
    typedef std::shared_ptr<ExtendedSVM> Ptr;

    inline CvSVMDecisionFunc* get_decision_function()
    {
        return decision_func;
    }

    inline double rho() const
    {
        if(decision_func != nullptr)
            return decision_func->rho;
        else
            return std::numeric_limits<double>::quiet_NaN();
    }

    inline void print_decision_func()
    {
        std::cout << "alpha: [";
        for(int i = 0 ; i < decision_func->sv_count - 1; ++i) {
            std::cout << decision_func->alpha[i] << ", ";
        }
        std::cout << decision_func->alpha[decision_func->sv_count - 1]
                << "]" << std::endl;
        std::cout << "rho: " << decision_func->rho  * -1 << std::endl;
    }

   inline void export_decision_func(cv::FileStorage &fs) const
    {
        fs << "svm_alpha" << "[";
        for(int i = 0 ; i < decision_func->sv_count ; ++i)
            fs << decision_func->alpha[i];
        fs << "]";
        fs << "svm_rho" << -decision_func->rho;
    }

    inline void set_parameters(const cv::SVMParams &params)
    {
        set_params(params);
    }
};
}


#endif // EXTENDED_SVM_HPP
