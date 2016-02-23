/// HEADER
#include <csapex_evaluation/confusion_matrix.h>

/// SYSTEM
#include <limits>

using namespace csapex;

ConfusionMatrix::ConfusionMatrix()
    : threshold(std::numeric_limits<double>::quiet_NaN())
{
//    initializeClass(0);
//    initializeClass(1);
//    initializeClass(2);
//    initializeClass(3);
//    initializeClass(4);
}

void ConfusionMatrix::reportClassification(int actual, int prediction)
{
    if(classes_set.count(actual) == 0) {
        initializeClass(actual);
    }
    if(classes_set.count(prediction) == 0) {
        initializeClass(prediction);
    }

    ++histogram[std::make_pair(actual, prediction)];
}

void ConfusionMatrix::resetClass(int _class)
{
    for(std::vector<int>::const_iterator it = classes.begin(); it != classes.end(); ++it) {
        histogram[std::make_pair(*it, _class)] = 0;
        histogram[std::make_pair(_class, *it)] = 0;
    }
}

void ConfusionMatrix::initializeClass(int _class)
{
    classes.push_back(_class);
    classes_set.insert(_class);

    resetClass(_class);
}

void ConfusionMatrix::reset()
{
    for(auto c: classes) {
        resetClass(c);
    }
}
