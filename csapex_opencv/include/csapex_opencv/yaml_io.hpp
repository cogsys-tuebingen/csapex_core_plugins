/// COMPONENT
#include <csapex_opencv/roi.h>

/// SYSTEM
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

/// YAML
namespace YAML {
template<>
struct convert<cv::KeyPoint> {
    static Node encode(const cv::KeyPoint& rhs)
    {
        Node node;
        node["pt"] = rhs.pt;
        node["size"] = rhs.size;
        node["angle"] = rhs.angle;
        node["response"] = rhs.response;
        node["octave"] = rhs.octave;
        node["class_id"] = rhs.class_id;

        return node;
    }

    static bool decode(const Node& node, cv::KeyPoint& rhs)
    {
        if(!node.IsMap()) {
            return false;
        }

        rhs.pt = node["pt"].as<cv::Point2f>();
        rhs.size = node["size"].as<float>();
        rhs.angle = node["angle"].as<float>();
        rhs.response = node["response"].as<float>();
        rhs.octave = node["octave"].as<int>();
        rhs.class_id = node["class_id"].as<int>();

        return true;
    }
};


template<typename T>
struct convert<cv::Point_<T> > {
    typedef cv::Point_<T> Point;

    static Node encode(const Point& rhs)
    {
        Node node;
        node["x"] = rhs.x;
        node["y"] = rhs.y;

        return node;
    }

    static bool decode(const Node& node, Point& rhs)
    {
        if(!node.IsMap()) {
            return false;
        }

        rhs.x = node["x"].as<T>();
        rhs.y = node["y"].as<T>();

        return true;
    }
};

template<typename T, int size>
struct convert<cv::Vec<T, size> > {
    typedef cv::Vec<T, size> Vec;

    static Node encode(const Vec& rhs)
    {
        Node node;
        for(std::size_t i = 0; i < size; ++i) {
            node.push_back(rhs[i]);
        }

        return node;
    }

    static bool decode(const Node& node, Vec& rhs)
    {
        if(!node.IsSequence()) {
            return false;
        }

        for(std::size_t i = 0; i < size; ++i) {
            rhs[i] = node[i].as<T>();
        }

        return true;
    }
};

template<>
struct convert<cv::DMatch> {
    static Node encode(const cv::DMatch& rhs)
    {
        Node node;
        node["distance"] = rhs.distance;
        node["imgIdx"] = rhs.imgIdx;
        node["queryIdx"] = rhs.queryIdx;
        node["trainIdx"] = rhs.trainIdx;

        return node;
    }

    static bool decode(const Node& node, cv::DMatch& rhs)
    {
        if(!node.IsMap()) {
            return false;
        }

        rhs.distance = node["distance"].as<float>();
        rhs.imgIdx = node["imgIdx"].as<int>();
        rhs.queryIdx = node["queryIdx"].as<int>();
        rhs.trainIdx = node["trainIdx"].as<int>();

        return true;
    }
};

template<>
struct convert<cv::Mat> {
    // TODO: implement more efficiently!
    static Node encode(const cv::Mat& rhs)
    {
        Node node;
        node["version"] = 2;

        cv::FileStorage fs(".yml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
        fs << "data" << rhs;

        std::string data = fs.releaseAndGetString();
        node["data"] = YAML::Binary((unsigned char*) data.data(), data.size());

        return node;
    }

    static bool decode(const Node& node, cv::Mat& rhs)
    {
        if(!node.IsMap()) {
            return false;
        }

        int version = 1;
        if(node["version"].IsDefined()) {
            version = node["version"].as<int>();
        }

        switch(version) {
        case 1:
        {
            std::string buffer = node["data"].as<std::string>();
            cv::FileStorage fs(buffer, cv::FileStorage::READ | cv::FileStorage::MEMORY);
            fs["data"] >> rhs;
            return true;
        }
        case 2:
        {
            YAML::Binary data = node["data"].as<YAML::Binary>();
            std::string buffer(data.data(), data.data() + data.size());
//            buffer.swap(data.data());
            cv::FileStorage fs(buffer, cv::FileStorage::READ | cv::FileStorage::MEMORY);
            fs["data"] >> rhs;
            return true;
        }
        default:
            break;
        }

        throw std::runtime_error(std::string("unknown serialization version ") + std::to_string(version));
    }
};


template<>
struct convert<csapex::Roi> {
    static Node encode(const csapex::Roi& rhs)
    {
        Node node;
        node["x"] = rhs.x();
        node["y"] = rhs.y();
        node["w"] = rhs.w();
        node["h"] = rhs.h();
        return node;
    }

    static bool decode(const Node& node, csapex::Roi& rhs)
    {
        if(!node.IsMap()) {
            return false;
        }

        int x = node["x"].as<int>();
        int y = node["y"].as<int>();
        int w = node["w"].as<int>();
        int h = node["h"].as<int>();

        rhs = csapex::Roi(x,y,w,h);
        return true;
    }
};

}
