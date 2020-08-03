#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <iomanip>

#include "BSplineSurface.h"
#include "nlopt.hpp"

using namespace std;

typedef struct {
    double x, y, z;
    BSplineSurface surface;
} opt_data;

double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    opt_data *d = (opt_data *) data;
    array<double,3> r = {0.0, 0.0, 0.0};
    array<double,3> n_hat = {0.0, 0.0, 0.0};
    array<double,3> r_u = {0.0, 0.0, 0.0};
    array<double,3> r_v = {0.0, 0.0, 0.0};
    d->surface.calculateSurfacePoint(x[0],x[1],r,n_hat,r_u,r_v);
    
    return (r[0]-d->x)*(r[0]-d->x)+(r[1]-d->y)*(r[1]-d->y)+(r[2]-d->z)*(r[2]-d->z);
}

int main(){
    BSplineSurface temp;
    temp.loadSurface("table");
    array<double,3> r = {0.0, 0.0, 0.0};
    array<double,3> n_hat = {0.0, 0.0, 0.0};
    array<double,3> r_u = {0.0, 0.0, 0.0};
    array<double,3> r_v = {0.0, 0.0, 0.0};
    temp.calculateSurfacePoint(0.001, 0.001,r,n_hat,r_u,r_v);

    // Where is the point
    cout << "R: " << r[0] << " " << r[1] << " " << r[2] << endl;

    // // Now try and find the same point using NLOPT
    // nlopt::opt opt(nlopt::LN_COBYLA, 2);
    // std::vector<double> lb(2);
    // lb[0] = 0.0; lb[1] = 0.0;
    // std::vector<double> ub(2);
    // ub[0] = 1.0; ub[1] = 1.0;

    // opt.set_lower_bounds(lb);
    // opt.set_upper_bounds(ub);

    

    // std::vector<double> x(2);
    // x[0] = 0.3; x[1] = 0.4;
    // double minf;

    // opt_data data;
    // data.x=r[0]; data.y=r[1]; data.z=r[2];
    // data.surface = temp;

    // opt.set_min_objective(myfunc,&data);
    // opt.set_xtol_rel(1e-4);

    // try{
    //     nlopt::result result = opt.optimize(x, minf);
    //     std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
    //         << std::setprecision(6) << minf << std::endl;
    // }
    // catch(std::exception &e) {
    //     std::cout << "nlopt failed: " << e.what() << std::endl;
    // }

    return 0;
}