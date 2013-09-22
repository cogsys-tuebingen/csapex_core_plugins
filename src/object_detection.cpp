/// HEADER
#include "object_detection.h"

/// PROJECT
#include <csapex/model/box.h>
#include <csapex/model/connector_in.h>
#include <csapex/model/connector_out.h>
#include <csapex_vision/cv_mat_message.h>
#include <csapex/utility/qt_helper.hpp>
#include <utils/extractor.h>
#include <utils/extractor_factory.h>
#include <utils/extractor_manager.h>

#include <csapex_vision_features/keypoint_message.h>

/// SYSTEM
#include <pluginlib/class_list_macros.h>

// Register the class
PLUGINLIB_EXPORT_CLASS(csapex::ObjectDetection, csapex::BoxedObject)

using namespace csapex;
using namespace connection_types;

ObjectDetection::ObjectDetection()
    : in_a_(NULL),
      in_b_(NULL),
      in_c_(NULL),
      in_d_(NULL),
      out_(NULL),
      btn_(NULL),
      has_a_(false),
      has_b_(false),
      has_c_(false),
      has_d_(false),
      publish_a_(true),
      container_sliders_(NULL)
{
    // Insert this plug-in into the "Vision" category.
    // Create the category, if it doesn't exist.
    Tag::createIfNotExists("Vision");
    addTag(Tag::get("Vision"));
    hessianThreshold = 400;
}

void ObjectDetection::fill(QBoxLayout *layout)
{
    // Connector for the first input image (sub id = 0)
    in_a_ = new ConnectorIn(box_, 0);
    in_a_->setLabel("Object");
    in_a_->setType(connection_types::CvMatMessage::make());

    // Connector for the second input image (sub id = 1)
    in_b_ = new ConnectorIn(box_, 1);
    in_b_->setLabel("Mask 1");
    in_b_->setType(connection_types::CvMatMessage::make());

    // Connector for the second input image (sub id = 2)
    in_c_ = new ConnectorIn(box_, 2);
    in_c_->setLabel("Scene");
    in_c_->setType(connection_types::CvMatMessage::make());

    // Connector for the second input image (sub id = 3)
    in_d_ = new ConnectorIn(box_, 3);
    in_d_->setLabel("Mask 2");
    in_d_->setType(connection_types::CvMatMessage::make());

    // Connector for the output input image (sub id = 0)
    out_ = new ConnectorOut(box_, 0);
    out_->setLabel("output");
    out_->setType(csapex::connection_types::KeypointMessage::make());

    // Register the connectors
    box_->addInput(in_a_);
    box_->addInput(in_b_);
    box_->addInput(in_c_);
    box_->addInput(in_d_);
    box_->addOutput(out_);

    // Combobox to choose between different keydetectors
    ExtractorManager& manager = ExtractorManager::instance();

    detectorbox_ = new QComboBox;
    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    foreach(Pair fc, manager.featureDetectors()) {
        std::string key = fc.second.getType();
        detectorbox_->addItem(key.c_str());

        state.params[key] = ExtractorManager::instance().featureDetectorParameters(key);
    }

    layout->addLayout(QtHelper::wrap("Detector", detectorbox_));

    extractorbox_ = new QComboBox();
    extractorbox_->addItem("SURF");
    layout->addLayout(QtHelper::wrap("Extractor", extractorbox_));

    matcherbox_ = new QComboBox();
    matcherbox_->addItem("SURF");
    layout->addLayout(QtHelper::wrap("Matcher", matcherbox_));

    opt = new QFrame;
    opt->setLayout(new QVBoxLayout);
    layout->addWidget(opt);

    //Connect Combobox to Signal
    QObject::connect(detectorbox_, SIGNAL(currentIndexChanged(int)), this , SLOT(updateDetector(int)));

    QObject::connect(box_, SIGNAL(placed()), this, SIGNAL(modelChanged()));
}

void ObjectDetection::updateSliders(){
    hessianThreshold = hessian_slider_->value();
}

void ObjectDetection::updateDetector(int slot){
    state.key = detectorbox_->currentText().toStdString();

    QtHelper::clearLayout(opt->layout());
    QBoxLayout* layout = dynamic_cast<QBoxLayout*> (opt->layout());
    assert(layout);

    foreach(QObject* cb, callbacks) {
        delete cb;
    }
    callbacks.clear();

    foreach(const vision::Parameter& para, state.params[state.key]) {
        std::string name = para.name();

        if(para.is<int>()) {
            QSlider* slider = QtHelper::makeSlider(layout, name , para.as<int>(), para.min<int>(), para.max<int>());
            slider->setValue(para.as<int>());

            boost::function<void()> cb = boost::bind(&ObjectDetection::updateParam<int>, this, name, boost::bind(&QSlider::value, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(para.is<double>()) {
            QDoubleSlider* slider = QtHelper::makeDoubleSlider(layout, name , para.as<double>(), para.min<double>(), para.max<double>(), para.step<double>());
            slider->setDoubleValue(para.as<double>());

            boost::function<void()> cb = boost::bind(&ObjectDetection::updateParam<double>, this, name, boost::bind(&QDoubleSlider::doubleValue, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(para.is<bool>()) {
            QCheckBox* box = new QCheckBox;
            box->setChecked(para.as<bool>());

            layout->addLayout(QtHelper::wrap(name, box));

            boost::function<void()> cb = boost::bind(&ObjectDetection::updateParam<bool>, this, name, boost::bind(&QCheckBox::isChecked, box));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks.push_back(call);

            QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

        } else {
            opt->layout()->addWidget(new QLabel((name + "'s type is not yet implemented").c_str()));
        }
    }


    update();
}

template <typename T>
void ObjectDetection::updateParam(const std::string& name, T value)
{
    BOOST_FOREACH(vision::Parameter& para, state.params[state.key]) {
        if(para.name() == name) {
            para.set<T>(value);

            change = true;
            Q_EMIT guiChanged();

            return;
        }
    }
}

void ObjectDetection::updateDynamicGui(QBoxLayout *layout){
    updateDetector(0);
}

void ObjectDetection::updateModel()
{
    if(change) {
        change = false;
        update();
    }
}

void ObjectDetection::update()
{
    Extractor::Ptr next = ExtractorFactory::create(state.key, "", vision::StaticParameterProvider(state.params[state.key]));

    QMutexLocker lock(&extractor_mutex);
    extractor = next;
}

void ObjectDetection::messageArrived(ConnectorIn *source)
{
    if(source == in_a_) {
        has_a_ = true;
    } else if(source == in_b_) {
        has_b_ = true;
    }
    if(source == in_c_) {
        has_c_ = true;
    } else if(source == in_d_) {
        has_d_ = true;
    }


    if(!extractor) {
        setError(true, "no extractor set");
        return;
    }

    if(change) {
        return;
    }

    bool use_mask = in_b_->isConnected();
    if(has_a_ && (has_b_ || !use_mask)) {
        setError(false);

        has_a_ = false;
        has_c_ = false;

        ConnectionType::Ptr msg = in_a_->getMessage();
        CvMatMessage::Ptr img_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);

        ObjectDetection::Ptr key_msg(new KeypointMessage);

        {
            QMutexLocker lock(&extractor_mutex);
            if(use_mask) {
                ConnectionType::Ptr msg = in_b_->getMessage();
                CvMatMessage::Ptr mask_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);

                extractor->extractKeypoints(img_msg->value, mask_msg->value, key_msg->value);

            } else {
                extractor->extractKeypoints(img_msg->value, cv::Mat(), key_msg->value);
            }
        }

        out_->publish(key_msg);
    }


    ////////////////////////////////////////////////////////////////


    // One of the two connectors has received a message, find out which
//    if(source == in_a_) {
//        has_a_ = true;
//    } else if(source == in_c_) {
//        has_c_ = true;
//    }
//    // Make sure that we have both images
//    if(has_a_ && has_c_) {
//        has_a_ = has_c_ = false;

//        // Publish the selected image
//        CvMatMessage::Ptr img_msg_a = boost::dynamic_pointer_cast<CvMatMessage> (in_a_->getMessage());
//        CvMatMessage::Ptr mask_msg_a = boost::dynamic_pointer_cast<CvMatMessage> (in_b_->getMessage());
//        CvMatMessage::Ptr img_msg_b = boost::dynamic_pointer_cast<CvMatMessage> (in_c_->getMessage());
//        CvMatMessage::Ptr mask_msg_b = boost::dynamic_pointer_cast<CvMatMessage> (in_d_->getMessage());

//        if(img_msg_a.get() && !img_msg_a->value.empty() && img_msg_b.get() && !img_msg_b->value.empty()) {
//            if(!mask_msg_a.get()) {
//                mask_msg_a.reset(new CvMatMessage);
//            }
//            if(!mask_msg_b.get()) {
//                mask_msg_b.reset(new CvMatMessage);
//            }
//            Surfhomography surfhomo;
//            CvMatMessage::Ptr img_msg_result(new CvMatMessage);
//            img_msg_result->value = surfhomo.calculation(img_msg_a->value, img_msg_b->value,mask_msg_a->value, mask_msg_b->value,hessianThreshold);
//            out_->publish(img_msg_result);
//        }

//    }
}
Memento::Ptr ObjectDetection::getState() const
{
    return boost::shared_ptr<State>(new State(state));
}

void ObjectDetection::setState(Memento::Ptr memento)
{
    boost::shared_ptr<ObjectDetection::State> m = boost::dynamic_pointer_cast<ObjectDetection::State> (memento);
    assert(m.get());

    //    state = *m;
    state.key = m->key;

    typedef std::pair<std::string, std::vector<vision::Parameter> > Pair;
    foreach(Pair pair, m->params) {
        foreach(const vision::Parameter& para, pair.second) {
            std::vector<vision::Parameter>& target = state.params[pair.first];
            BOOST_FOREACH(vision::Parameter& existing_param, target) {
                if(existing_param.name() == para.name()) {
                    existing_param.setFrom(para);
                }
            }
        }
    }

    int slot = 0;
    for(int i = 0, n = detectorbox_->count(); i < n; ++i) {
        if(detectorbox_->itemText(i).toStdString() == state.key) {
            slot = i;
            break;
        }
    }
    detectorbox_->setCurrentIndex(slot);
}

void ObjectDetection::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "key" << YAML::Value << key;
    out << YAML::Key << "params" << YAML::Value << params;
}
void ObjectDetection::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("params")) {
        node["params"] >> params;
    }
    if(node.FindValue("key")) {
        node["key"] >> key;
    }
}
