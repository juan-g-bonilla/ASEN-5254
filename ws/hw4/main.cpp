// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the headers for HW4 code
#include "CSpaceSkeleton.h"
#include "ManipulatorSkeleton.h"

using namespace amp;

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    MyManipulator2D manipulator({0.5, 1, 0.5});

    // You can visualize your manipulator given an angle state like so:
    amp::ManipulatorState test_state(3);
    test_state << M_PI / 6, M_PI / 3, M_PI * 7. / 4.;


    // std::cout << manipulator.getJointLocation(test_state, 3);



    manipulator.getLinkLengths() = {1, 0.3};
    amp::ManipulatorState ik_state = manipulator.getConfigurationFromIK({0.8, 0.8});
    std::cout << manipulator.getJointLocation(ik_state, 2);

    // The visualizer uses your implementation of forward kinematics to show the joint positions so you can use that to test your FK algorithm
    Visualizer::makeFigure(manipulator, ik_state); 

    // // Create the collision space constructor
    std::size_t n_cells = 300;
    MyManipulatorCSConstructor cspace_constructor(n_cells);

    // // // Create the collision space using a given manipulator and environment
    // MyManipulator2D manipulator2({1, 1});
    // std::unique_ptr<amp::GridCSpace2D> cspace = cspace_constructor.construct(manipulator2, HW4::getEx3Workspace1());

    // // // You can visualize your cspace 
    // Visualizer::makeFigure(*cspace);

    // Visualizer::showFigures();

    // // Grade method
    amp::HW4::grade<MyManipulator2D>(cspace_constructor, "Juan.GarciaBonilla@colorado.edu", argc, argv);
    return 0;
}