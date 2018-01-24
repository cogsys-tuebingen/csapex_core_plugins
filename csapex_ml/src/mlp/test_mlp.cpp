#include "mlp.h"
#include <iostream>
#include <chrono>

namespace tt {
using clock_t    = std::chrono::high_resolution_clock;
using time_t     = clock_t::time_point;
using duration_t = clock_t::duration;

inline double milliseconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() * 1e-6;
}

inline double microseconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() * 1e-3;
}

inline int64_t nanoseconds(const duration_t &duration)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}
}

int main(int argc, char *argv[])
{

    std::string file = argv[1];
    std::cout << "loading mlp config: " << file << std::endl;

    mlp::MLPConfig cfg;
    cfg.load(file);

    mlp::MLP mlp(cfg);

    std::vector<double> input = {3.14167, -0.0658774, 1.57096, 13.9641, 1.57083, -0.315142,
                                 0.629466, -0.0012423, 1.88457, -0.770979, 1.19953, -0.0111944};
    std::vector<double> output = {0.1918923572609873, 1.4568250390267559, -0.343192582289692,
                                  -0.10181160599688983, -0.03369011544420859, -0.1856456301301303};

    std::vector<double> response(6);

    tt::time_t start = tt::clock_t::now();
    for(int i = 0; i < 10000; ++i){

        mlp.compute(input.data(),response.data());
    }

    std::cout << "Writing " << tt::milliseconds(tt::clock_t::now() - start) << "ms" << std::endl;
    std::cout << "Writing " << tt::milliseconds(tt::clock_t::now() - start) / 10000 << "ms per call" << std::endl;

    auto it = output.begin();
    std::size_t i = 0;
    for(double val : response){
        if(val - *it < 1e-5){
            std::cout  << "test " << i << " passed" << std::endl;
        } else {
            std::cout  << "test " << i << " passed" << std::endl;
        }
        ++it;
        ++i;
    }



    return 0;
}
