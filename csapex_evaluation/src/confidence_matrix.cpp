/// HEADER
#include <csapex_evaluation/confidence_matrix.h>

using namespace csapex;

ConfidenceMatrix::ConfidenceMatrix(UpdateType update) : update_type(update)
{
}

void ConfidenceMatrix::reportConfidence(int truth, int estimation, float estimation_confidence)
{
    if (classes_set.count(truth) == 0) {
        initializeClass(truth);
    }
    if (classes_set.count(estimation) == 0) {
        initializeClass(estimation);
    }

    float& value = confidences[std::make_pair(truth, estimation)];
    int& updates = confidence_updates[std::make_pair(truth, estimation)];

    switch (update_type) {
        case ARGMAX:
            value = std::max(value, estimation_confidence);
            ++updates;
            break;
        case MEAN:
            value = (value * updates + estimation_confidence) / (float)(updates + 1);
            ++updates;
            break;
        default:
            break;
    }
}

void ConfidenceMatrix::resetClass(int _class)
{
    for (std::vector<int>::const_iterator it = classes.begin(); it != classes.end(); ++it) {
        confidences[std::make_pair(*it, _class)] = 0.f;
        confidences[std::make_pair(_class, *it)] = 0.f;
        confidence_updates[std::make_pair(*it, _class)] = 0;
        confidence_updates[std::make_pair(_class, *it)] = 0;
    }
}

void ConfidenceMatrix::initializeClass(int _class)
{
    classes.push_back(_class);
    classes_set.insert(_class);

    resetClass(_class);
}

void ConfidenceMatrix::reset(UpdateType update)
{
    for (auto c : classes) {
        resetClass(c);
    }

    update_type = update;
}
