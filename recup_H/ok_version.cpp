#include <tubex.h>
#include <tubex-pyibex.h>
#include <tubex-rob.h>
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

    // Actual trajectories (state + derivative)
    TrajectoryVector v_truth(3);
    TrajectoryVector x_truth(3);

    v_truth[2] = u + n_u;
    x_truth[2] = v_truth[2].primitive() + x0[2];
    v_truth[0] = 10*cos(x_truth[2]);
    v_truth[1] = 10*sin(x_truth[2]);
    x_truth[0] = v_truth[0].primitive() + x0[0];
    x_truth[1] = v_truth[1].primitive() + x0[1];

    //------------------------------------------------

    Tube tu(u,dt);

    Tube m_u(x_truth[2],dt);
    m_u.inflate(0.03); //bruit de mesure

    IntervalVector v_x0({{0,0},{0,0},{2,2}});

    // Actual trajectories (state + derivative)
    TubeVector tv_truth(tdomain, dt, 3); 
    TubeVector tx_truth(tdomain, dt, 3); 

    tv_truth[2] = tu + n_u;
    tx_truth[2] = m_u; // On a mesuré l'orientation du robot
    tv_truth[0] = 10*cos(tx_truth[2]);
    tv_truth[1] = 10*sin(tx_truth[2]);
    tx_truth[0] = tv_truth[0].primitive() + v_x0[0];
    tx_truth[1] = tv_truth[1].primitive() + v_x0[1];

    
    

    //------------------------------------------------------------------------------------------------------------
    
    vibes::beginDrawing();
    VIBesFigMap fig_map("Estimation de la position");
    fig_map.set_properties(100, 100, 600, 300);

    fig_map.add_tube(&tx_truth, "tx", 0, 1);
    fig_map.smooth_tube_drawing(true);

    fig_map.add_trajectory(&x_truth, "x*", 0, 1); // tracé de la trajectoire et du robot
    

    fig_map.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map.show(0.5); // Deuxième affichage après contraction

    vibes::endDrawing();


    //------------------------------------------------------------------------------------------------------------
    // Perceiving landmarks and solving a SLAM
    //------------------------------------------------------------------------------------------------------------

    //Landmarks
    // Vector b1({6, 12});
    // Vector b2({-2, -5});
    // Vector b3({-3, 20});
    // Vector b4({3, 4});

    // vector<IntervalVector> v_map;

    // v_map.push_back(IntervalVector(b1));
    // v_map.push_back(IntervalVector(b2));
    // v_map.push_back(IntervalVector(b3));
    // v_map.push_back(IntervalVector(b4));


    vector<Vector> v_map({{6, 12},{-2, -5},{-3, 20},{3, 4}});

    //Observations
    vector<IntervalVector> v_obs;

    // création des observations toutes les 0.1s
    for (float t = tdomain.lb(); t <= tdomain.ub(); t += 2 * dt)
    {
        int j = rand() % v_map.size(); //indice de la landmark choisi aléatoirement
        double dx = v_map[j][0] - x_truth(t)[0];
        double dy = v_map[j][1] - x_truth(t)[1];
        Interval range = sqrt(sqr(dx) + sqr(dy)); //distance mesurée

        v_obs.push_back(IntervalVector({Interval(t), range.inflate(0.03), Interval(j)})); // t,d avec incertitude,i
    }


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
    for (int i=0;i<v_map.size();i++)
    {   
        for (int j=0;j<v_obs.size();j++)
        {
            if (v_obs[j][2].mid() == i)
            {
                cn.add(ctc::dist, {v_pi[j][0],v_pi[j][1],m[i], v_obs[j][1]});
            }
        }
    }



    // évaluation de pi = x(ti)
    for (int i=0;i<v_obs.size();i++)
    {
        cn.add(ctc::eval, {v_obs[i][0],v_pi[i],tx_truth,tv_truth});
        //cn.add(ctc::eval, {v_obs[i][0],v_pi[i],tube_x,tube_v});
        //cn.add(ctc::eval, {v_obs[i][0],v_pi[i],x1_x2, dx1_dx2});
    }

    //---------------------------------------------------------------------------------------------------
    // AFFICHAGE AVEC VIBES
    //---------------------------------------------------------------------------------------------------
   
    vibes::beginDrawing();

    VIBesFigMap fig_map4("SLAM");
    fig_map4.set_properties(100, 100, 600, 300);
    fig_map4.smooth_tube_drawing(true);

    fig_map4.add_tube(&tx_truth, "tx", 0, 1); //Tube avant contraction

    fig_map4.add_trajectory(&x_truth, "x*", 0, 1); // tracé de la trajectoire et du robot
    
    for(const auto& b : v_map)
    {
        fig_map4.add_beacon(b, 0.2); // drawing beacons
    }



    fig_map4.axis_limits(-2.5,2.5,-0.1,0.1, true);

    cn.contract();
    fig_map4.show(0.5);

    for(auto& mi : m)
    {
        fig_map4.draw_box(mi, "#FE9A2E[#FE9A2E77]");
    }

    vibes::endDrawing();

}