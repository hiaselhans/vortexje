#include <vortexje/bindings/case.hpp>


using namespace std;
using namespace Eigen;
using namespace Vortexje;

// Run a test for a single angle of attack:
static const double pi = 3.141592653589793238462643383279502884;

// Main:
VortexjeCase::VortexjeCase(vectorlist& nodes, vectorlist& panels)
{
    // Enable wake convection:
    Parameters::convect_wake = true;

    // Create lifting surface object:
    this->wing = std::make_shared<LiftingSurface>("main");

    for (auto node: nodes) {
        this->wing->add_node(node);
    }
    
    // Load airfoil data:
    int trailing_edge_point_id;
    vector<Vector3d, Eigen::aligned_allocator<Vector3d> > clarky_airfoil;
    

    const int n_airfoils = 21;
    
    const double chord = 1.0;
    const double span = 5.0;
    
    vector<int> prev_airfoil_nodes;
    
    vector<vector<int> > node_strips;
    vector<vector<int> > panel_strips;
    
    for (int i = 0; i < n_airfoils; i++) {
        vector<Vector3d, Eigen::aligned_allocator<Vector3d> > airfoil_points;
        for (int j = 0; j < (int) clarky_airfoil.size(); j++)
            airfoil_points.push_back(Vector3d(chord * clarky_airfoil[j](0), chord * clarky_airfoil[j](1), i * span / (double) (n_airfoils - 1)));
             
        vector<int> airfoil_nodes = surface_builder.create_nodes_for_points(airfoil_points);
        node_strips.push_back(airfoil_nodes);
        
        if (i > 0) {
            vector<int> airfoil_panels = surface_builder.create_panels_between_shapes(airfoil_nodes, prev_airfoil_nodes, trailing_edge_point_id);
            panel_strips.push_back(airfoil_panels);
        }
            
        prev_airfoil_nodes = airfoil_nodes;
    }

    surface_builder.finish(node_strips, panel_strips, trailing_edge_point_id);
    
    // Translate into the canonical coordinate system:
    Vector3d translation(-chord / 3.0, 0.0, -span / 2.0);
    wing->translate(translation);
    
    // Prescribe angle of attack:
    double alpha = 5.0 / 180.0 * pi;
    wing->rotate(Vector3d::UnitZ(), -alpha);

    this->body = std::make_shared<Body>(std::string("wing-section"));
    this->body->add_lifting_surface(this->wing);
    this->solver.add_body(this->body);
    
    Vector3d freestream_velocity(30, 0, 0);
    this->solver.set_freestream_velocity(freestream_velocity);
    
    double fluid_density = 1.2;
    this->solver.set_fluid_density(fluid_density);
}

int VortexjeCase::run() {
    
    // Run simulation:
    double t = 0.0;
    double dt = 0.01;
    int step_number = 0;
    
    this->solver.initialize_wakes(dt);
    while (t < 60) {
        // Solve:
        this->solver.solve(dt);
        
        // Enable below to log the velocity field:
        //field_writer.write_velocity_field(solver, "velocity-field.vtk", -0.5, 2.0, -3.0, 3.0, -1.0, 1.0, 0.2, 0.2, 0.2);
        
        // Update wake:
        this->solver.update_wakes(dt);
        
        // Step time:
        t += dt;
        step_number++;
    }
    
}