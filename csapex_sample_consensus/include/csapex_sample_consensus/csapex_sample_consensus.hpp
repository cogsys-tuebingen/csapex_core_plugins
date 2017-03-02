#ifndef CSAPEX_SAMPLE_CONSENSUS_HPP
#define CSAPEX_SAMPLE_CONSENSUS_HPP

//// model base types
#include "models/sac_model.hpp"
#include "models/sac_model_from_normals.hpp"

//// discrete implementations of models
#include "models/sac_model_plane.hpp"
#include "models/sac_model_normal_plane.hpp"
#include "models/sac_model_parallel_normal_plane.hpp"

//// algorithm base type
#include "algorithms/sac.hpp"
#include "algorithms/antsac.hpp"
#include "algorithms/ransac.hpp"

#endif // CSAPEX_SAMPLE_CONSENSUS_HPP
