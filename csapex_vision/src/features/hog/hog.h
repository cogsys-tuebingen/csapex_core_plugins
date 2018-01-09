#ifndef HOG_HPP
#define HOG_HPP
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/
//HOG descriptor length = #Blocks * #CellsPerBlock * #BinsPerCell
//                      = (64/8-1) * (128/8-1) * (2*2) * 9
//                      = 7        * 15        *  4    * 9
//                      = 3780

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace csapex {
//////////////// HOG (Histogram-of-Oriented-Gradients) Descriptor and Object Detector //////////////

//! struct for detection region of interest (ROI)
struct DetectionROI
{
    //! scale(size) of the bounding box
    double scale;
    //! set of requrested locations to be evaluated
    std::vector<cv::Point> locations;
    //! vector that will contain confidence values for each location
    std::vector<double> confidences;
};

class SimilarRects
{
public:
    SimilarRects(double _eps) : eps(_eps) {}
    inline bool operator()(const cv::Rect& r1, const cv::Rect& r2) const
    {
        double delta = eps*(std::min(r1.width, r2.width) + std::min(r1.height, r2.height))*0.5;
        return std::abs(r1.x - r2.x) <= delta &&
            std::abs(r1.y - r2.y) <= delta &&
            std::abs(r1.x + r1.width - r2.x - r2.width) <= delta &&
            std::abs(r1.y + r1.height - r2.y - r2.height) <= delta;
    }
    double eps;
};

template<bool doCorrect>
struct GammaCorrection
{
};

template<>
struct GammaCorrection<false>
{
    template<typename T>
    static inline T apply(const T value)
    {
        return value;
    }
};

template<>
struct GammaCorrection<true>
{
    template<typename T>
    static inline T apply(const T value)
    {
        return sqrt(value);
    }
};

struct HOGDescriptor
{
public:
    enum { L2Hys = 0
         };
    enum { DEFAULT_NLEVELS = 64
         };

    HOGDescriptor() : winSize(64,128), blockSize(16,16), blockStride(8,8),
        cellSize(8,8), nbins(9), derivAperture(1), winSigma(-1),
        histogramNormType(HOGDescriptor::L2Hys), L2HysThreshold(0.2), gammaCorrection(true),
        free_coef(-1.f), nlevels(HOGDescriptor::DEFAULT_NLEVELS), signedGradient(false)
    {}

    HOGDescriptor(cv::Size _winSize, cv::Size _blockSize, cv::Size _blockStride,
                  cv::Size _cellSize, int _nbins, int _derivAperture=1, double _winSigma=-1,
                  int _histogramNormType=HOGDescriptor::L2Hys,
                  double _L2HysThreshold=0.2, bool _gammaCorrection=false,
                  int _nlevels=HOGDescriptor::DEFAULT_NLEVELS, bool _signedGradient=false)
        : winSize(_winSize), blockSize(_blockSize), blockStride(_blockStride), cellSize(_cellSize),
          nbins(_nbins), derivAperture(_derivAperture), winSigma(_winSigma),
          histogramNormType(_histogramNormType), L2HysThreshold(_L2HysThreshold),
          gammaCorrection(_gammaCorrection), free_coef(-1.f), nlevels(_nlevels), signedGradient(_signedGradient)
    {}

    HOGDescriptor(const std::string& filename)
    {
        load(filename);
    }

    HOGDescriptor(const HOGDescriptor& d)
    {
        d.copyTo(*this);
    }

    virtual ~HOGDescriptor() {}

    size_t getDescriptorSize() const;
    bool checkDetectorSize() const;
    double getWinSigma() const;

    virtual void setSVMDetector(const std::vector<float> &_svmdetector);

    virtual bool read(cv::FileNode& fn);
    virtual void write(cv::FileStorage& fs, const std::string& objname) const;

    virtual bool load(const std::string& filename, const std::string& objname = "");
    virtual void save(const std::string& filename, const std::string& objname = "") const;
    virtual void copyTo(HOGDescriptor& c) const;

    virtual void computeSingle(const cv::Mat &img,
                               std::vector<float> &descriptors);

    virtual void compute(const cv::Mat &img,
                         std::vector<float>& descriptors,
                         cv::Size winStride = cv::Size(), cv::Size padding = cv::Size(),
                         const std::vector<cv::Point>& locations = std::vector<cv::Point>()) const;

    virtual bool classify(const cv::Mat &img,
                          const double hitThreshold, double &weight);


    virtual bool classify(const cv::Mat &img,
                          const double hitThreshold,
                          std::vector<float> &positive_svm_weights, std::vector<float> &negative_svm_weights,
                          std::vector<float> &descriptor,
                          double &weight);

    //! with found weights output
    virtual void detect(const cv::Mat& img,
                        std::vector<cv::Point>& foundLocations,
                        std::vector<double>& weights,
                        double hitThreshold = 0,
                        cv::Size winStride = cv::Size(),
                        cv::Size padding = cv::Size(),
                        const std::vector<cv::Point>& searchLocations = std::vector<cv::Point>()) const;
    //! without found weights output
    virtual void detect(const cv::Mat& img,
                        std::vector<cv::Point>& foundLocations,
                        double hitThreshold = 0,
                        cv::Size winStride = cv::Size(),
                        cv::Size padding = cv::Size(),
                        const std::vector<cv::Point>& searchLocations=std::vector<cv::Point>()) const;

    //! with result weights output
    virtual void detectMultiScale(const cv::Mat &img,
                                  std::vector<cv::Rect>& foundLocations,
                                  std::vector<double>& foundWeights,
                                  double hitThreshold = 0,
                                  cv::Size winStride = cv::Size(),
                                  cv::Size padding = cv::Size(),
                                  double scale = 1.05,
                                  double finalThreshold = 2.0,
                                  bool useMeanshiftGrouping = false) const;
    //! without found weights output
    virtual void detectMultiScale(const cv::Mat &img,
                                  std::vector<cv::Rect>& foundLocations,
                                  double hitThreshold = 0,
                                  cv::Size winStride = cv::Size(),
                                  cv::Size padding = cv::Size(),
                                  double scale = 1.05,
                                  double finalThreshold = 2.0,
                                  bool useMeanshiftGrouping = false) const;

    virtual void computeGradientDefault(const cv::Mat& img,  cv::Mat& grad,  cv::Mat& angleOfs,
                                 cv::Size paddingTL = cv::Size(), cv::Size paddingBR = cv::Size()) const;


    template<typename T, bool gamma>
    void computeGradientGeneric(const cv::Mat& img,  cv::Mat& grad,  cv::Mat& angleOfs,
                                cv::Size paddingTL = cv::Size(), cv::Size paddingBR = cv::Size()) const;

    static std::vector<float> getDefaultPeopleDetector();
    static std::vector<float> getDaimlerPeopleDetector();

    cv::Size winSize;
    cv::Size blockSize;
    cv::Size blockStride;
    cv::Size cellSize;
    int nbins;
    int derivAperture;
    double winSigma;
    int histogramNormType;
    double L2HysThreshold;
    bool gammaCorrection;
    std::vector<float> svmDetector;
    float free_coef;
    int nlevels;
    bool signedGradient;

    //! evaluate specified ROI and return confidence value for each location
    virtual void detectROI(const cv::Mat& img, const std::vector<cv::Point> &locations,
                           std::vector<cv::Point>& foundLocations,
                           std::vector<double>& confidences,
                           double hitThreshold = 0,
                           cv::Size winStride = cv::Size(),
                           cv::Size padding = cv::Size()) const;

    //! evaluate specified ROI and return confidence value for each location in multiple scales
    virtual void detectMultiScaleROI(const cv::Mat& img,
                                     std::vector<cv::Rect>& foundLocations,
                                     std::vector<DetectionROI>& locations,
                                     double hitThreshold = 0,
                                     int groupThreshold = 0) const;

    //! read/parse Dalal's alt model file
    void readALTModel(const std::string &modelfile);
    void groupRectangles(std::vector<cv::Rect>& rectList, std::vector<double>& weights, int groupThreshold, double eps) const;

    static void groupRectangles_meanshift(std::vector<cv::Rect>& rectList,
                                          std::vector<double>& foundWeights,
                                          std::vector<double>& scales,
                                          double detectThreshold = 0.0,
                                          cv::Size winDetSize = cv::Size(64, 128))
    {
        groupRectangles_meanshift(rectList, detectThreshold, &foundWeights, scales, winDetSize);
    }

    static void groupRectangles_meanshift(std::vector<cv::Rect>& rectList,
                                          double detectThreshold,
                                          std::vector<double>* foundWeights,
                                          std::vector<double>& scales,
                                          cv::Size winDetSize);


    static void clipObjects(cv::Size sz, std::vector<cv::Rect>& objects,
                            std::vector<int>* a, std::vector<double>* b);

    static void groupRectangles(std::vector<cv::Rect>& rectList, int groupThreshold, double eps,
                                std::vector<int>* weights, std::vector<double>* levelWeights);
};
}
#endif // HOG_HPP
