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

int main()
{
    //------------------------------------------------------------------------------------------------------------
    // INITIALIZATION
    //------------------------------------------------------------------------------------------------------------

    double dt = 0.05;
    double tf = 15;
    Interval tdomain(0,tf); // temporal domain [t0,tf]

    //------------------------------------------------------------------------------------------------------------
    
    // Initial pose x0=(0,0,2)
    Vector x0({0,0,2});

    // System input
    Trajectory u(tdomain, TFunction("3*(sin(t)^2)+t/100"), dt);

    RandTrajectory n_u(tdomain, dt, Interval(-0.03,0.03));
    RandTrajectory n_x3 = n_u;

    // Actual trajectories (state + derivative)
    TrajectoryVector v_truth(3);
    TrajectoryVector x_truth(3);

    v_truth[2] = u + n_u;
    x_truth[2] = v_truth[2].primitive() + x0[2] + n_x3;
    v_truth[0] = 10*cos(x_truth[2]);
    v_truth[1] = 10*sin(x_truth[2]);
    x_truth[0] = v_truth[0].primitive() + x0[0];
    x_truth[1] = v_truth[1].primitive() + x0[1];

    //------------------------------------------------

    Tube m_u(x_truth[2],dt);
    m_u.inflate(0.03); //bruit de mesure

    IntervalVector v_x0({{0,0},{0,0},{2,2}});

    // Actual trajectories (state + derivative)
    TubeVector tx_truth(3, Tube(tdomain, dt, Interval()));
    TubeVector tv_truth(3, Tube(tdomain, dt, Interval()));

    tv_truth[2] = Tube(u + n_u,dt).inflate(0.03);
    tx_truth[2] = m_u; // On a mesuré l'orientation du robot
    tv_truth[0] = 10*cos(tx_truth[2]);
    tv_truth[1] = 10*sin(tx_truth[2]);
    tx_truth[0] = tv_truth[0].primitive() + x0[0];
    tx_truth[1] = tv_truth[1].primitive() + x0[1];


    //------------------------------------------------------------------------------------------------------------
    // Perceiving landmarks and solving a SLAM
    //------------------------------------------------------------------------------------------------------------

    //Landmarks
    vector<Vector> v_map({{6, 12},{-2, -5},{-3, 20},{3, 4}});

    vector<IntervalVector> v_map2;

    v_map2.push_back(IntervalVector(v_map[0]));
    v_map2.push_back(IntervalVector(v_map[1]));
    v_map2.push_back(IntervalVector(v_map[2]));
    v_map2.push_back(IntervalVector(v_map[3]));



    // Generating observations obs=(t,range,bearing) of these landmarks
    int max_nb_obs = 150;
    Interval visi_range(0,4); // [0m,75m]
    Interval visi_angle(-M_PI,M_PI); // frontal sonar

    TrajectoryVector traj_x({x_truth[0],x_truth[1]});

    vector<IntervalVector> v_obs = DataLoader::generate_observations(traj_x, v_map2, max_nb_obs,true, visi_range, visi_angle);

    // vector<IntervalVector> v_obs;

    // // Generate observations every 2*dt
    // for (float t = tdomain.lb(); t <= tdomain.ub(); t += 2 * dt)
    // {
    //     int i = rand() % v_map.size();
    //     double dx = v_map[i][0] - x_truth(t)[0];
    //     double dy = v_map[i][1] - x_truth(t)[1];
    //     Interval range = sqrt(sqr(dx) + sqr(dy));
    //     v_obs.push_back(IntervalVector({Interval(t), range.inflate(0.03), Interval(i)})); // instant t of the measurement, landmark i observed, range, bearing
    // }

    // ---------------------------------------------------------------------------------------------------
    // PREPARATION DES CONTRAINTES
    // ---------------------------------------------------------------------------------------------------

    // Initialisation des intermédiaires pi

    vector<IntervalVector> v_pi;
    for (int i=0;i<v_obs.size();i++)
    {
        v_pi.push_back(IntervalVector(3));
    }
    //---------------------------------------------------------------------------------------------------

    vector<IntervalVector> m(v_map.size(), IntervalVector(2));

    //---------------------------------------------------------------------------------------------------
    // RESEAU DE CONTRACTIONS
    //---------------------------------------------------------------------------------------------------

    ContractorNetwork cn; //Création du réseau de contracteurs

    // contrainte sur yi --> yi = gi(p)
    for (int i=0;i<v_obs.size();i++)
    {   
        IntervalVector &p = cn.create_dom(IntervalVector(3));

        int j = v_obs[i][2].mid(); // associated landmark
        cn.add(ctc::dist, {m[j],p[0],p[1], v_obs[i][1]});

        cn.add(ctc::eval, {v_obs[i][0],p,tx_truth,tv_truth});

    }

    //---------------------------------------------------------------------------------------------------
    // AFFICHAGE AVEC VIBES
    //---------------------------------------------------------------------------------------------------
   
    vibes::beginDrawing();
    VIBesFigMap fig_map2("SLAM");
    fig_map2.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map2.set_properties(100, 100, 600, 300);
    fig_map2.smooth_tube_drawing(true);

    fig_map2.add_tube(&tx_truth, "tx", 0, 1); //Tube avant contraction
    fig_map2.add_trajectory(&x_truth, "x*", 0, 1); // tracé de la trajectoire et du robot
    
    for(const auto& b : v_map)
    {
        fig_map2.add_beacon(b, 0.2); // drawing beacons
    }

    fig_map2.show(0.5);

    cn.contract();
    fig_map2.show(0.5);

    for(auto& mi : m)
    {
        fig_map2.draw_box(mi, "red[]");
    }

    vibes::endDrawing();

}