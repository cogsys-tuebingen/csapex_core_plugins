#ifndef MLP_H
#define MLP_H

#include <cstddef>
#include <memory>

namespace mlp {

double exp(const double& arg);
double tanh(const double& arg);
double sigmoid(const double& arg);
double relu(const double& arg);

class MLP {

public:
    typedef std::shared_ptr<MLP> Ptr;

    MLP     (const size_t input_size, const size_t layers_num, const size_t* layer_sizes, const bool* layer_bias, const size_t weights_num, const double* weights);
	virtual ~MLP();

	void compute(const double* input, double* output);

private:
	const size_t  layers_num;
	const size_t* layer_sizes;
    const bool*   layer_bias;

	double* buffer;
	size_t  buffer_size;

	size_t  input_size;
	size_t  output_size;

	size_t weights_num;
	double* weights;

};


}




#endif
