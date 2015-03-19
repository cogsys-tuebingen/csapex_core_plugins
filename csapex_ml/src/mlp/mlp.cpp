#include "mlp.h"
#include <math.h>


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



MLP::MLP (
	const size_t layers_num, 
	const size_t* layer_sizes,
	const size_t weights_num,
	const double* weights
) : 
	layers_num(layers_num), 
	layer_sizes(layer_sizes),
	weights_num(weights_num)
{
	//
	this->weights = new double[this->weights_num];
	//
	for (size_t i = 0; i < this->weights_num; ++i) {
		this->weights[i] = weights[i];
	}
	//
	this->input_size  = this->layer_sizes[0];
	this->output_size = this->layer_sizes[this->layers_num - 1];
	//
	this->buffer_size = 0;
	//
	for (size_t i = 0; i < layers_num; ++i) {
		this->buffer_size += layer_sizes[i];
	}
	//
	this->buffer = new double[this->buffer_size];
	//
}

MLP::~MLP() {
	//
	delete[] this->buffer;
	delete[] this->weights;
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
	ptr = this->buffer;
	//
	for (size_t i = 0; i < this->input_size; ++i) {
		*ptr = *input_ptr;
		ptr++;
		input_ptr++;
	}
	//
	size_t curr_size;
	double* curr_ptr;
	//
	const double* weight_ptr = this->weights;
	//
	size_t prev_size = input_size;
	double* prev_ptr  = this->buffer;
	//
	// for all hidden layer.
	//
	for (size_t h = 1;  h < (this->layers_num - 1); ++h) {
		curr_size = this->layer_sizes[h];
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
				read_ptr++;
				weight_ptr++;
			}

			//
			// activate
			//
			*ptr = tanh(*ptr);
			ptr++;
		}
		//
		prev_size = curr_size;
		prev_ptr = curr_ptr;
	}
	//
	// for output layer.
	//
	curr_size = this->layer_sizes[this->layers_num - 1];
	curr_ptr  = prev_ptr + prev_size;
	//
	// collect netinputs and netinput sum.
	//
	double softmax_sum = 0;
	ptr = curr_ptr;
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
			const double xwij = *read_ptr * *weight_ptr;
			*ptr += xwij;
			read_ptr++;
			weight_ptr++;
		}
		//
		const double x = exp(*ptr);
		softmax_sum += x;
		*ptr = x;
		ptr++;
	}
	//
	const double inv_softmax_sum = 1.0 / softmax_sum;
	//
	// softmax activation and copy output.
	//
	ptr = curr_ptr;
	//
	for (size_t j = 0; j < curr_size; ++j) {
		//
		*ptr = *ptr * inv_softmax_sum;
		ptr++;
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
	for (size_t i = 0; i < this->buffer_size; ++i) {
		std::cout << buffer[i] << std::endl;
	}
	*/
}





}
