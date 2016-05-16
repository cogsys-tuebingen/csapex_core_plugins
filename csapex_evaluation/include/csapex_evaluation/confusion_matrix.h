#ifndef CONFUSION_MATRIX_H
#define CONFUSION_MATRIX_H

/// SYSTEM
#include <vector>
#include <set>
#include <map>

namespace csapex
{

class ConfusionMatrix
{
public:
    ConfusionMatrix();
    void reportClassification(int truth, int estimation);

    void reset();
    void initializeClass(int _class);

public:
    std::vector<int> classes;
    std::map<int, std::string> class_names;
    std::map<std::pair<int, int>, int> histogram;


    double threshold;

private:
    void resetClass(int _class);

private:
    std::set<int> classes_set;
};

}

#endif // CONFUSION_MATRIX_H

