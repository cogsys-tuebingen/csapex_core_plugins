#ifndef CONFIDENCE_MATRIX_H
#define CONFIDENCE_MATRIX_H

/// SYSTEM
#include <map>
#include <set>
#include <vector>

namespace csapex
{
class ConfidenceMatrix
{
public:
    enum UpdateType
    {
        ARGMAX,
        MEAN
    };

    ConfidenceMatrix(UpdateType update = MEAN);
    void reportConfidence(int truth, int estimation, float estimation_confidence);

    void reset(UpdateType update = MEAN);

public:
    std::vector<int> classes;
    std::map<std::pair<int, int>, float> confidences;
    std::map<std::pair<int, int>, int> confidence_updates;
    UpdateType update_type;

private:
    void resetClass(int _class);
    void initializeClass(int _class);

private:
    std::set<int> classes_set;
};

}  // namespace csapex

#endif  // CONFIDENCE_MATRIX_H
