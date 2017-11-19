#ifndef FEATURE_CLASSIFICATION_MAPPING_HPP
#define FEATURE_CLASSIFICATION_MAPPING_HPP
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <csapex_ml/features_message.h>
class FeatureClassificationMapping
{
public:
    FeatureClassificationMapping() {}

    bool forward(const csapex::connection_types::FeaturesMessage& in,
                 csapex::connection_types::FeaturesMessage& out)
    {
        int label = in.classification;
        auto it = std::find_if(mapping_.begin(), mapping_.end(),
                               [&label](const std::pair<int, int>& p){
            return p.first == label; });
        bool success = false;
        if(it != mapping_.end()){
            out.classification = it->second;
            success = true;

        }
        return success;
    }

    bool inverse(const csapex::connection_types::FeaturesMessage& in,
                 csapex::connection_types::FeaturesMessage& out)
    {
        int label = in.classification;
        auto it = std::find_if(mapping_.begin(), mapping_.end(),
                               [&label](const std::pair<int, int>& p){
            return p.second == label; });

        bool success = false;
        if(it != mapping_.end()){
            out.classification = it->first;
            success = true;
        }
        return success;
    }

    void addRule(int from, int to)
    {
        std::pair<int,int> p(from,to);
        auto it = std::find(mapping_.begin(), mapping_.end(), p);
        if(it == mapping_.end()){
            mapping_.push_back(p);
        }
    }

    std::vector<std::pair<int,int>>::iterator findRule(int from, int to)
    {
        std::pair<int,int> p(from,to);
        return std::find(mapping_.begin(), mapping_.end(), p);
    }

    std::size_t size() const
    {
        return mapping_.size();
    }

    void clear()
    {
        mapping_.clear();
    }

    void save(std::string file_name) const
    {
        std::ofstream file(file_name);
        YAML::Emitter yamlEmit(file);
        YAML::Node doc;

        for(auto val : mapping_){
            YAML::Node pNode;
            pNode["from"] = val.first;
            pNode["to"] = val.second;
            doc["feature_mapping"].push_back(pNode);
        }
        yamlEmit << doc;
    }

    bool load(std::string file_name)
    {
        mapping_.clear();
        YAML::Node doc = YAML::LoadFile(file_name);
        auto tmp = doc["feature_mapping"];
        if(!tmp.IsDefined())
        {
            return false;
        }
        for(auto it = tmp.begin(); it != tmp.end(); ++it){
            auto pNode = *it;
            int o = pNode["from"].as<int>();
            int n = pNode["to"].as<int>();
            std::pair<int,int> tmp(o,n);
            mapping_.push_back(tmp);
        }
        return true;
    }
private:
    std::vector<std::pair<int,int>> mapping_;

};
#endif // FEATURE_CLASSIFICATION_MAPPING_HPP
