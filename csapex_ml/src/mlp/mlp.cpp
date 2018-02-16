#include "mlp.h"
#include <math.h>
#include <fstream>
#include <iterator>
#include <sstream>

// TODOS:
// use other activation functions than relu
// implement softmax layer

namespace mlp {

double exp(const double& arg) {
    return ::exp(arg);
}

double tanh(const double& arg) {
    return ::tanh(arg);
}

double sigmoid(const double& arg) {
    return 1.0 / (1.0 + exp(-arg));
}


double relu(const double& arg) {
    return (arg > 0)?(arg):(0.0);
}

template<char delimiter>
class LayerDelimitedBy : public std::string
{};

bool MLPConfig::load(const std::string &path)
{
    std::ifstream in( path );

    std::string line;
    std::getline(in, line);
    if(line != "network:"){
        return false;
    }
    // Read Networkconfig
    std::getline(in, line);
    std::vector<std::string> layers;

    size_t pos = 0;
    std::string delimiter = "-";
    while ((pos = line.find(delimiter)) != std::string::npos) {
        std::string token = line.substr(0, pos);
        layers.push_back(token);
        line.erase(0, pos + delimiter.length());
    }

    layers.push_back(line);

    if(layers.front() != "MLP"){
        return false;
    }

    layers.erase(layers.begin());

    input_size = std::stod(layers.front());
    layers.erase(layers.begin());

    layers_num = layers.size();

    while(!layers.empty()){
        std::string layer = layers.front();

        // bias layer?
        char&  bias = layer.at(layer.size() -1);
        if(bias == 'b'){
            layer.erase(layer.size() -1,1);
            layer_bias.push_back(true);
        } else{
            layer_bias.push_back(false);
        }


        std::size_t pos_e = layer.find(exp_layer);
        std::size_t pos_h = layer.find(tanh_layer);
        std::size_t pos_s = layer.find(sigmoid_layer);
        std::size_t pos_r = layer.find(relu_layer);
        std::size_t pos_l = layer.find(linear_layer);
        std::size_t pos_so = layer.find(softmax_layer);

        if(pos_e != std::string::npos){
            layer.erase(0, exp_layer.size());
            function = ActivationFunction::EXP;
        } else if(pos_h != std::string::npos){
            layer.erase(0, tanh_layer.size());
            function = ActivationFunction::TANH;
        } else if(pos_s != std::string::npos ){
            layer.erase(0, sigmoid_layer.size());
            function = ActivationFunction::SIGMOID;
        } else if(pos_r != std::string::npos){
            layer.erase(0, relu_layer.size());
            function = ActivationFunction::RELU;
        } else if(pos_l != std::string::npos){
            layer.erase(0, linear_layer.size());
            output_type = OutputType::LINEAR;
        } else if(pos_so != std::string::npos){
            layer.erase(0, softmax_layer.size());
            output_type = OutputType::SOFTMAX;
        }

        std::size_t layer_size = std::stoul(layer);
        layer_sizes.push_back(layer_size);
        layers.erase(layers.begin());
    }

    //empty space
    std::getline(in, line);

    while(line != "weights:"){
        if(!std::getline(in,line)){
            return false;
        }
    }

    // weights
    std::getline(in, line);
    std::istringstream weights_str(line);


    weights = std::vector<double>( ( std::istream_iterator<double>( weights_str ) ),
                                   std::istream_iterator<double>() );

    weights_num = weights.size();

    return true;

}

MLP::MLP(const MLPConfig &config) :
    layers_num_(config.layers_num),
    layer_sizes_(config.layer_sizes),
    layer_bias_(config.layer_bias),
    input_size_(config.input_size),
    output_size_(layer_sizes_.back()),
    weights_num_(config.weights_num),
    weights_(config.weights),
    out_type_(config.output_type),
    act_func_(config.function)
{
    initialize();

}

MLP::MLP (const size_t input_size,
        const size_t layers_num,
        const std::vector<size_t> &layer_sizes,
        const std::vector<bool> &layer_bias,
        const size_t weights_num,
        const std::vector<double> &weights
        ) :
    layers_num_(layers_num),
    layer_sizes_(layer_sizes),
    layer_bias_(layer_bias),
    input_size_(input_size),
    output_size_(layer_sizes_.back()),
    weights_num_(weights_num),
    weights_(weights)
{
    initialize();
}

void MLP::initialize()
{
    buffer_size_ = input_size_;

    for (auto sz : layer_sizes_) {
        buffer_size_ += sz;
    }

    buffer_ = std::vector<double>(buffer_size_);

}


void MLP::setOutputType(const OutputType &type)
{
    out_type_ = type;
}

void MLP::setActivationFunction(const ActivationFunction &func_type)
{
    act_func_ = func_type;
}

void MLP::compute(const double* input, double* output) {
    //
    const double* input_ptr  = input;
    //
    const double* read_ptr;
    double* ptr;
    //
    // copy input into buffer
    //
    ptr = buffer_.data();
    //
    for (size_t i = 0; i < input_size_; ++i) {
        *ptr = *input_ptr;
        ptr++;
        input_ptr++;
    }
    //
    size_t curr_size;
    double* curr_ptr;
    //
    const double* weight_ptr = weights_.data();
    //
    size_t prev_size = input_size_;
    double* prev_ptr  = buffer_.data();
    //
    // for all hidden layer + output layer.
    //
    const size_t output_layer = layers_num_ - 1;
    double softmax_sum = 0;
    //
    for (size_t h = 0;  h < (layers_num_); ++h) {
        //
        curr_size = layer_sizes_[h];
        curr_ptr  = prev_ptr + prev_size;
        ptr       = curr_ptr;
        //
        for (size_t j = 0; j < curr_size; ++j) {
            //
            read_ptr = prev_ptr;
            //
            // reset activation.
            //
            *ptr = 0;
            //
            // collect net input.
            //
            for (size_t i = 0; i < prev_size; ++i) {
                const double xwij = (*read_ptr) * (*weight_ptr);
                *ptr += xwij;
                ++read_ptr;
                ++weight_ptr;
            }
            if (layer_bias_[h]) {
                const double wbj = *weight_ptr;
                *ptr += wbj;
                ++weight_ptr;
            }
            //
            // activate
            //
            if (h < output_layer) {
                *ptr = activationFunction(*ptr);
            } else if (h == output_layer && out_type_ == OutputType::SOFTMAX){ // softmax layer
                const double x = activationFunction(*ptr);
                softmax_sum += x;
                *ptr = x;
            }
            //
            ptr++;
        }
        //
        prev_size = curr_size;
        prev_ptr = curr_ptr;
    }

    //
    // softmax activation and copy output.
    //
    if (out_type_ == OutputType::SOFTMAX){
    const double inv_softmax_sum = 1.0 / softmax_sum;
        ptr = curr_ptr;
        for (size_t j = 0; j < curr_size; ++j) {
            //
            *ptr = *ptr * inv_softmax_sum;
            ptr++;
        }
    }


    //
    // copy output.
    //
    double* output_ptr = output;
    read_ptr = curr_ptr;
    //
    for (size_t j = 0; j < curr_size; ++j) {
        *output_ptr = *read_ptr;
        output_ptr++;
        read_ptr++;
    }
    //
    // done.
    //
    /*
    for (size_t i = 0; i < buffer_size; ++i) {
        std::cout << buffer[i] << std::endl;
    }
    */
}


double MLP::activationFunction(const double &arg)
{
    double res = 0;
    switch (act_func_) {
    case ActivationFunction::EXP:
        res = exp(arg);
        break;
    case ActivationFunction::TANH:
        res = tanh(arg);
        break;
    case ActivationFunction::SIGMOID:
        res = sigmoid(arg);
        break;
    case ActivationFunction::RELU:
        res = relu(arg);
        break;
    }
    return res;
}


}
