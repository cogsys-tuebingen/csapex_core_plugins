/// HEADER
#include <csapex_vision/roi.h>

/// PROJECT
#include <csapex/utility/assert.h>

using namespace csapex;

Roi::Roi()
{

}

Roi::Roi(cv::Rect rectangle, cv::Scalar color, int classification)
    : rect_(rectangle), color_(color), classification_(classification)
{
    check();
}

Roi::Roi(int x, int y, int width, int height, cv::Scalar color, int classification)
    : rect_(x, y, width, height), color_(color), classification_(classification)
{
    check();
}

void Roi::setLabel(const std::string &label)
{
    label_ = label;
}

std::string Roi::label() const
{
    return label_;
}

void Roi::check()
{
    apex_assert_hard(rect_.x >= 0);
    apex_assert_hard(rect_.y >= 0);
}

bool Roi::covers(const Roi& rhs) const
{
    if(rect_ == rhs.rect_ || rect_ == (rect_ | rhs.rect_)) {
        /// rhs is equal to roi
        ///  or contained in roi
        return true;
    }

    return false;
}

void Roi::grow(int pixels_x, int pixels_y)
{
    rect_ = rect_ - cv::Point(pixels_x / 2, pixels_y / 2) + cv::Size(pixels_x, pixels_y);
}

int Roi::x() const
{
    return rect_.x;
}


int Roi::y() const
{
    return rect_.y;
}


int Roi::w() const
{
    return rect_.width;
}


int Roi::h() const
{
    return rect_.height;
}

int Roi::classification() const
{
    return classification_;
}

cv::Rect Roi::rect() const
{
    return rect_;
}

cv::Scalar Roi::color() const
{
    return color_;
}
