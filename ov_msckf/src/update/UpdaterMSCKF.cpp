#include "UpdaterMSCKF.h"



using namespace ov_core;
using namespace ov_msckf;





void UpdaterMSCKF::update(State *state, std::vector<Feature*>& feature_vec) {




    // 1. clean all feature measurements and make sure they all have valid clone times




    // 2. try to triangulate all MSCKF or new SLAM features that have measurements




    // 3. compute linear system for each feature, nullspace project, and reject




    // 4. with all good features update the state





}



