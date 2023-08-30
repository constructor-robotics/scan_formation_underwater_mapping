#include <image_registration.h>
#include <opencv2/core/eigen.hpp>


fourierMellinRegistration::fourierMellinRegistration(const cv::Mat &im) :
    rows_(im.rows), cols_(im.cols), log_polar_size_(std::max(rows_, cols_)),
    imdft_(rows_, cols_), imdft_logpolar_(log_polar_size_, log_polar_size_),
    image_transforms_(rows_, cols_, log_polar_size_, log_polar_size_)
{
    high_pass_filter_ = imdft_.getHighPassFilter();
    initialize(im);
}

fourierMellinRegistration::~fourierMellinRegistration()
{
}

void fourierMellinRegistration::initialize(const cv::Mat &im)
{
    processImage(im, im0_gray_, im0_logpolar_);
}

void fourierMellinRegistration::registerImage(const cv::Mat &im, cv::Mat &registered_image, std::vector<double> &transform_params, bool display_images)
{
    double rs_row, rs_col;
    double t_row, t_col;
    double scale, rotation;

    processImage(im, im1_gray_,im1_logpolar_);

    imdft_logpolar_.phaseCorrelate(im1_logpolar_, im0_logpolar_, rs_row, rs_col);
    image_transforms_.getScaleRotation(rs_row, rs_col, scale, rotation);
    image_transforms_.rotateAndScale(im0_gray_, im0_rotated_, scale, rotation);

    transform_params[2] = rotation;
    //transform_params[3] = scale;
    transform_params[3] = 1.0;

    if (display_images)
    {
        cv::imshow("im0_rotated", im0_rotated_);
    }

    imdft_.phaseCorrelate(im1_gray_, im0_rotated_, t_row, t_col);
    image_transforms_.translate(im0_rotated_, registered_image, t_col, t_row); // x, y

    transform_params[0] = t_col;
    transform_params[1] = t_row;

    if (display_images)
    {
        cv::imshow("im0_registered", registered_image);
    }
}

void fourierMellinRegistration::next()
{
    im1_logpolar_.copyTo(im0_logpolar_);
    im1_gray_.copyTo(im0_gray_);
}

cv::Mat fourierMellinRegistration::getBorderMask()
{
    return image_transforms_.getBorderMask();
}

void fourierMellinRegistration::processImage(const cv::Mat &im, cv::Mat &gray, cv::Mat &log_polar)
{
    im.convertTo(gray, CV_32F, 1.0/255.0);
    cv::cvtColor(gray, gray, cv::COLOR_BGR2GRAY);

    cv::Mat apodized;
    cv::Mat im_dft_cv;

    image_transforms_.apodize(gray, apodized);
    Eigen::MatrixXf im_dft = (imdft_.fftShift(imdft_.fft(apodized)).cwiseProduct(high_pass_filter_).cwiseAbs()).cast<float>();
    cv::eigen2cv(im_dft, im_dft_cv);
    image_transforms_.remapLogPolar(im_dft_cv, log_polar);
}

cv::Mat fourierMellinRegistration::getPreviousImage()
{
    return im0_gray_;
}

cv::Mat fourierMellinRegistration::getCurrentImage()
{
    return im1_gray_;
}
