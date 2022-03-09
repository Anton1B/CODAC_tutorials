#include <tubex.h>
#include <tubex-pyibex.h>
#include <vector>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <cstring>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace tubex;

/*
Exercice C1

g1 = x1 + y1*cos(x3+y2) - m1
g1 = x1-m1 + y1*cos(x3+y2)
g1 = -d1 + y1*cos(theta)

g2 = x2 + y1*sin(x3+y2) - m2
g2 = x2-m2 + y1*sin(x3+y2)
g2 = -d2 * y1*sin(theta)

Or, g(x,y) =0
Donc 
d1 = y1*cos(theta)
d2 = y1*sin(theta)
*/


int main()
{
    // Truth (unknown pose)
    Vector x_truth({2.,1.,M_PI/6.}); // actual state vector
    Vector y_truth({6.,M_PI/6.});
    Vector m_truth({5.,6.2});


    // bounded sets related to the state, the measurement and the landmark position

    IntervalVector x{{-oo,oo},{-oo,oo},{M_PI/6.,M_PI/6.}};

    IntervalVector y{{6.0,6.0},{M_PI/6.,M_PI/6.}};
    y[0].inflate(0.3);
    y[1].inflate(0.1);


    IntervalVector m{{5.0,5.0},{6.2,6.2}};
    m[0].inflate(0.2);
    m[1].inflate(0.2);

    Interval interval_theta = y[1]+x[2];
    Interval interval_rho = y[0];

    vibes::beginDrawing();

    VIBesFigMap fig_map("Map");
    fig_map.set_properties(100,100,500,500);
    fig_map.axis_limits(0,7,0,7);
    fig_map.draw_vehicle(x_truth, 1);
    fig_map.draw_box(m, "red");
    fig_map.draw_box(x.subvector(0,1)); // does not display anything if unbounded

    fig_map.draw_pie(x_truth[0], x_truth[1], interval_rho, interval_theta, "Gray");
    fig_map.draw_pie(x_truth[0], x_truth[1], (Interval(0.1)|interval_rho), interval_theta, "lightGray");


    Interval d1;
    Interval d2;
    Interval theta;

    // Déclaration des cotracteurs
    pyibex::CtcPolar ctc_polar;
    CtcFunction ctc_add(Function("d", "m", "x", "d-m+x"));
    CtcFunction ctc_theta(Function("t", "x", "y", "t-x-y"));

    // CONTRACTION
    ContractorNetwork cn;       // Creating a Contractor Network

    cn.add(ctc_add, {d1, m[0], x[0]});
    cn.add(ctc_add, {d2, m[1], x[1]});
    cn.add(ctc_theta, {theta, x[2], y[1]});
    cn.add(ctc_polar, {d1, d2, y[0], theta});
    
    cn.contract();

    cout << d1<< endl;
    cout << d2<< endl;
    cout << x << endl;

    fig_map.draw_box(x.subvector(0,1)); // does not display anything if unbounded

    fig_map.show();

    vibes::endDrawing();

}