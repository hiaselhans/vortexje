#include <vortexje/solver.hpp>
#include <vortexje/surface-builder.hpp>

#include <cmath>
#include <iostream>
#include <fstream>

#include <vortexje/solver.hpp>
#include <vortexje/lifting-surface-builder.hpp>
#include <vortexje/shape-generators/airfoils/naca4-airfoil-generator.hpp>
#include <vortexje/surface-writers/vtk-surface-writer.hpp>

using namespace Vortexje;

using vectorlist = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;


class VortexjeCase {
    public:
        VortexjeCase(vectorlist& nodes, vectorlist& panels);

        void build_wake(vectorlist& wake_nodes);
        int run();



    private:
        std::shared_ptr<LiftingSurface> wing;
        std::shared_ptr<Body> body;
        Solver solver;

};