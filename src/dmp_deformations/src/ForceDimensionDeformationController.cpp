#include "DeformationController.cpp"

using namespace std;
using namespace std::chrono;

class ForceDimensionDeformationController: public DeformationController{
    public:
        ForceDimensionDeformationController();
};

// Default Constructor
// TODO: override with filename for learned behavior

ForceDimensionDeformationController::ForceDimensionDeformationController(){
    inputDevice_velocity = {0.0, 0.0, 0.0};
    prev_var_x = 0.0; prev_var_y = 0.0; prev_var_z = 0.0;
    var_x_changing = false; var_y_changing = false; var_z_changing = false;
    trajectoryFile = "learneddmp.csv";
    cout << "I BE OVERRIDING THE CLASS YO" << endl;
}


int main(int argc, char **argv) {    
    ForceDimensionDeformationController controller;
    int success = controller.run_deformation_controller(argc,argv);
    return 0;
}
