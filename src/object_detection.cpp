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
#include <QSplitter>

#include <csapex_vision_features/keypoint_message.h>
#include <csapex_vision_features/descriptor_message.h>

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
      out_key(NULL),
      has_a_(false),
      has_b_(false),
      has_c_(false),
      has_d_(false)
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
    out_->setType(csapex::connection_types::CvMatMessage::make());

    out_key = new ConnectorOut(box_, 1);
    out_key->setLabel("Keypoints");
    out_key->setType(csapex::connection_types::KeypointMessage::make());

    // Register the connectors
    box_->addInput(in_a_);
    box_->addInput(in_b_);
    box_->addInput(in_c_);
    box_->addInput(in_d_);
    box_->addOutput(out_);
    box_->addOutput(out_key);

    // Combobox to choose between different keydetectors
    ExtractorManager& manager = ExtractorManager::instance();

    detectorbox_ = new QComboBox;
    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    foreach(Pair fc, manager.featureDetectors()) {
        std::string key = fc.second.getType();
        detectorbox_->addItem(key.c_str());

        state.params_detector[key] = ExtractorManager::instance().featureDetectorParameters(key);
    }

    layout->addLayout(QtHelper::wrap("Detector", detectorbox_));

    extractorbox_ = new QComboBox;
    typedef std::pair<std::string, ExtractorManager::ExtractorInitializer> Pair;
    foreach(Pair fc, manager.descriptorExtractors()) {
        std::string des = fc.second.getType();
        extractorbox_->addItem(fc.second.getType().c_str());

        state.params_extractor[des] = ExtractorManager::instance().featureDescriptorParameters(des);
    }
    layout->addLayout(QtHelper::wrap("Descriptor", extractorbox_));

    matcherbox_ = new QComboBox();
    matcherbox_->addItem("SURF");
    layout->addLayout(QtHelper::wrap("Matcher", matcherbox_));

    opt = new QFrame;
    opt->setLayout(new QVBoxLayout);
    layout->addWidget(opt);

    opt1 = new QFrame;
    opt1->setLayout(new QVBoxLayout);
    layout->addWidget(opt1);


    //Connect Combobox to Signal
    QObject::connect(detectorbox_, SIGNAL(currentIndexChanged(int)), this , SLOT(updateDetector(int)));
    QObject::connect(extractorbox_, SIGNAL(currentIndexChanged(int)), this , SLOT(updateExtractor(int)));

    QObject::connect(box_, SIGNAL(placed()), this, SIGNAL(modelChanged()));
}

void ObjectDetection::updateDetector(int slot){
    state.key = detectorbox_->currentText().toStdString();

    QtHelper::clearLayout(opt->layout());
    QBoxLayout* layout = dynamic_cast<QBoxLayout*> (opt->layout());
    assert(layout);

    foreach(QObject* cb, callbacks_detector) {
        delete cb;
    }
    callbacks_detector.clear();

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);

    layout->addWidget(new QLabel("Extractor:"));

    foreach(const vision::Parameter& para, state.params_detector[state.key]) {
        std::string name = para.name();

        if(para.is<int>()) {
            QSlider* slider = QtHelper::makeSlider(layout, name , para.as<int>(), para.min<int>(), para.max<int>());
            slider->setValue(para.as<int>());

            boost::function<void()> cb = boost::bind(&ObjectDetection::updateParam<int>, this, name, boost::bind(&QSlider::value, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks_detector.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(para.is<double>()) {
            QDoubleSlider* slider = QtHelper::makeDoubleSlider(layout, name , para.as<double>(), para.min<double>(), para.max<double>(), para.step<double>());
            slider->setDoubleValue(para.as<double>());

            boost::function<void()> cb = boost::bind(&ObjectDetection::updateParam<double>, this, name, boost::bind(&QDoubleSlider::doubleValue, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks_detector.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(para.is<bool>()) {
            QCheckBox* box = new QCheckBox;
            box->setChecked(para.as<bool>());

            layout->addLayout(QtHelper::wrap(name, box));

            boost::function<void()> cb = boost::bind(&ObjectDetection::updateParam<bool>, this, name, boost::bind(&QCheckBox::isChecked, box));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks_detector.push_back(call);

            QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

        } else {
            opt->layout()->addWidget(new QLabel((name + "'s type is not yet implemented").c_str()));
        }
    }


    update();
}

void ObjectDetection::updateExtractor(int slot){
    state.des = extractorbox_->currentText().toStdString();

    QtHelper::clearLayout(opt1->layout());
    QBoxLayout* layout = dynamic_cast<QBoxLayout*> (opt1->layout());
    assert(layout);

    foreach(QObject* cb, callbacks_extractor) {
        delete cb;
    }
    callbacks_extractor.clear();

    QFrame* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);

    layout->addWidget(new QLabel("Descriptor:"));

    foreach(const vision::Parameter& para, state.params_extractor[state.des]) {
        std::string name = para.name();

        if(para.is<int>()) {
            QSlider* slider = QtHelper::makeSlider(layout, name , para.as<int>(), para.min<int>(), para.max<int>());
            slider->setValue(para.as<int>());

            boost::function<void()> cb = boost::bind(&ObjectDetection::updateParamdet<int>, this, name, boost::bind(&QSlider::value, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks_extractor.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(para.is<double>()) {
            QDoubleSlider* slider = QtHelper::makeDoubleSlider(layout, name , para.as<double>(), para.min<double>(), para.max<double>(), para.step<double>());
            slider->setDoubleValue(para.as<double>());

            boost::function<void()> cb = boost::bind(&ObjectDetection::updateParamdet<double>, this, name, boost::bind(&QDoubleSlider::doubleValue, slider));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks_extractor.push_back(call);

            QObject::connect(slider, SIGNAL(valueChanged(int)), call, SLOT(call()));

        } else if(para.is<bool>()) {
            QCheckBox* box = new QCheckBox;
            box->setChecked(para.as<bool>());

            layout->addLayout(QtHelper::wrap(name, box));

            boost::function<void()> cb = boost::bind(&ObjectDetection::updateParamdet<bool>, this, name, boost::bind(&QCheckBox::isChecked, box));
            qt_helper::Call* call = new qt_helper::Call(cb);
            callbacks_extractor.push_back(call);

            QObject::connect(box, SIGNAL(toggled(bool)), call, SLOT(call()));

        } else {
            opt1->layout()->addWidget(new QLabel((name + "'s type is not yet implemented").c_str()));
        }
    }


    updatedet();
}

template <typename T>
void ObjectDetection::updateParamdet(const std::string& name, T value)
{
    BOOST_FOREACH(vision::Parameter& para, state.params_extractor[state.des]) {
        if(para.name() == name) {
            para.set<T>(value);

            changedet = true;
            Q_EMIT guiChanged();

            return;
        }
    }
}


template <typename T>
void ObjectDetection::updateParam(const std::string& name, T value)
{
    BOOST_FOREACH(vision::Parameter& para, state.params_detector[state.key]) {
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
    updateExtractor(0);
}

void ObjectDetection::updateModel()
{
    if(change) {
        change = false;
        update();
    }
    if(changedet) {
        changedet = false;
        updatedet();
    }
}

void ObjectDetection::update()
{
    Extractor::Ptr next = ExtractorFactory::create(state.key, "", vision::StaticParameterProvider(state.params_detector[state.key]));

    QMutexLocker lock(&extractor_mutex);
    extractor = next;
}

void ObjectDetection::updatedet()
{
    Extractor::Ptr next = ExtractorFactory::create("", state.des, vision::StaticParameterProvider(state.params_extractor[state.des]));
//            ExtractorFactory::create(state.des, "", vision::StaticParameterProvider(state.params_extractor[state.des]));
    QMutexLocker lock(&extractor_mutex);
    descriptor = next;
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
    if(!descriptor) {
        setError(true, "no descriptor set");
        return;
    }

    if(change) {
        return;
    }

    bool use_mask = in_b_->isConnected();
    if((has_a_ && (has_b_ || !use_mask )) && (has_c_ && (has_d_ || !use_mask))) {
        setError(false);

        has_a_ = false;
        has_c_ = false;

        ConnectionType::Ptr msg = in_a_->getMessage();
        CvMatMessage::Ptr img_msg_a = boost::dynamic_pointer_cast<CvMatMessage> (msg);
        ConnectionType::Ptr msg_c = in_c_->getMessage();
        CvMatMessage::Ptr img_msg_c = boost::dynamic_pointer_cast<CvMatMessage> (msg_c);

        KeypointMessage::Ptr key_msg_a(new KeypointMessage);
        KeypointMessage::Ptr key_msg_c(new KeypointMessage);
        DescriptorMessage::Ptr des_msg_a(new DescriptorMessage);
        DescriptorMessage::Ptr des_msg_c(new DescriptorMessage);


        {
            QMutexLocker lock(&extractor_mutex);
            if(use_mask) {
                ConnectionType::Ptr msg = in_b_->getMessage();
                CvMatMessage::Ptr mask_msg = boost::dynamic_pointer_cast<CvMatMessage> (msg);

                extractor->extractKeypoints(img_msg_a->value, mask_msg->value, key_msg_a->value);

            } else {
                extractor->extractKeypoints(img_msg_a->value, cv::Mat(), key_msg_a->value);
                extractor->extractKeypoints(img_msg_c->value, cv::Mat(), key_msg_c->value);
                descriptor->extractDescriptors(img_msg_a->value, key_msg_a->value, des_msg_a->value);
                descriptor->extractDescriptors(img_msg_c->value, key_msg_c->value, des_msg_c->value);
            }
        }

        Surfhomography surfhomo;
        CvMatMessage::Ptr img_msg_result(new CvMatMessage);
        img_msg_result->value = surfhomo.calculation(img_msg_a->value, img_msg_c->value,key_msg_a->value,key_msg_c->value, des_msg_a->value, des_msg_c->value);
        out_->publish(img_msg_result);
        out_key->publish(key_msg_c);

//        out_->publish(key_msg_c);
    }

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
    foreach(Pair pair, m->params_detector) {
        foreach(const vision::Parameter& para, pair.second) {
            std::vector<vision::Parameter>& target = state.params_detector[pair.first];
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

    state.des = m->des;

    typedef std::pair<std::string, std::vector<vision::Parameter> > Pair;
    foreach(Pair pair, m->params_extractor) {
        foreach(const vision::Parameter& para, pair.second) {
            std::vector<vision::Parameter>& target = state.params_extractor[pair.first];
            BOOST_FOREACH(vision::Parameter& existing_param, target) {
                if(existing_param.name() == para.name()) {
                    existing_param.setFrom(para);
                }
            }
        }
    }

    slot = 0;
    for(int i = 0, n = extractorbox_->count(); i < n; ++i) {
        if(extractorbox_->itemText(i).toStdString() == state.des) {
            slot = i;
            break;
        }
    }
    extractorbox_->setCurrentIndex(slot);

}

void ObjectDetection::State::writeYaml(YAML::Emitter& out) const {
    out << YAML::Key << "key" << YAML::Value << key;
    out << YAML::Key << "params" << YAML::Value << params_detector;
    out << YAML::Key << "des" << YAML::Value << des;
    out << YAML::Key << "params_extractor" << YAML::Value << params_extractor;
}
void ObjectDetection::State::readYaml(const YAML::Node& node) {
    if(node.FindValue("params")) {
        node["params"] >> params_detector;
    }
    if(node.FindValue("key")) {
        node["key"] >> key;
    }
    if(node.FindValue("params_extractor")) {
        node["params_extractor"] >> params_extractor;
    }
    if(node.FindValue("des")) {
        node["des"] >> des;
    }

}
