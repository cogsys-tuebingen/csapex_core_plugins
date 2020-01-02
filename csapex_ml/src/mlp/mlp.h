#ifndef MLP_H
#define MLP_H

#include <cstddef>
#include <memory>
#include <vector>

namespace mlp
{
const std::string exp_layer = "exp";
const std::string tanh_layer = "tanh";
const std::string sigmoid_layer = "sigmoid";
const std::string relu_layer = "relu";
const std::string linear_layer = "linear";
const std::string softmax_layer = "softmax";

double exp(const double& arg);
double tanh(const double& arg);
double sigmoid(const double& arg);
double relu(const double& arg);

enum ActivationFunction
{
    EXP = 0,
    TANH = 1,
    SIGMOID = 2,
    RELU = 3
};

enum OutputType
{
    LINEAR = 0,
    SOFTMAX = 1,
};

struct MLPConfig
{
    MLPConfig()
    {
    }

    std::size_t input_size;
    std::size_t layers_num;
    std::size_t weights_num;
    std::vector<std::size_t> layer_sizes;
    std::vector<bool> layer_bias;
    std::vector<double> weights;
    OutputType output_type;
    ActivationFunction function;

    bool load(const std::string& path);
};

class MLP
{
public:
    typedef std::shared_ptr<MLP> Ptr;

    MLP(const MLPConfig& mlp);
    MLP(const size_t input_size_, const size_t layers_num_, const std::vector<size_t>& layer_sizes_, const std::vector<bool>& layer_bias_,
        const std::vector<double>& weights_);

    void setOutputType(const OutputType& type);
    void setActivationFunction(const ActivationFunction& func_type);

    void compute(const double* input, double* output);

private:
    void initialize();
    double activationFunction(const double& arg);

private:
    const size_t layers_num_;
    const std::vector<size_t> layer_sizes_;
    const std::vector<bool> layer_bias_;

    size_t input_size_;
    size_t output_size_;

    std::vector<double> weights_;

    std::vector<double> buffer_;
    size_t buffer_size_;
    OutputType out_type_;
    ActivationFunction act_func_;
};

}  // namespace mlp

#endif
