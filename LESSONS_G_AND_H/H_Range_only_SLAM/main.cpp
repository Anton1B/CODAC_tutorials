#include <tubex.h>
#include <tubex-pyibex.h>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
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
    
    // Potition initiale du robot x0=(0,0,2)
    Vector x0({0,0,2});

    // Gestion de la trajectoire
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
    // Passage aux tubes 

    Tube m_u(x_truth[2],dt);
    m_u.inflate(0.03); //bruit de mesure

    IntervalVector v_x0({{0,0},{0,0},{2,2}});

    TubeVector tx(3, Tube(tdomain, dt, Interval()));
    TubeVector tv(3, Tube(tdomain, dt, Interval()));

    tv[2] = Tube(u + n_u,dt).inflate(0.03);
    tx[2] = m_u; // On a mesuré l'orientation du robot
    tv[0] = 10*cos(tx[2]);
    tv[1] = 10*sin(tx[2]);
    tx[0] = tv[0].primitive() + x0[0];
    tx[1] = tv[1].primitive() + x0[1];

    TubeVector tx2 = tx;
    TubeVector tv2 = tv;

    //------------------------------------------------------------------------------------------------------------
    // Perceiving landmarks and solving a SLAM
    //------------------------------------------------------------------------------------------------------------

    //Landmarks
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

    // Boites permettant d'estimer la position des landmarks
    vector<IntervalVector> m(v_map.size(), IntervalVector(2)); //intitilisées à -inf;+inf

    //---------------------------------------------------------------------------------------------------
    // RESEAU DE CONTRACTIONS
    //---------------------------------------------------------------------------------------------------

    ContractorNetwork cn; //Création du réseau de contracteurs

    int j; // indice j de la landmark;
    
    for (int i=0;i<v_obs.size();i++)
    {   
        IntervalVector &p = cn.create_dom(IntervalVector(3)); //intermédiaires p

        j = v_obs[i][2].mid(); // On récupère l'indice de la landmark
        cn.add(ctc::dist, {m[j],p[0],p[1], v_obs[i][1]}); // contrainte sur yi --> yi = gi(p)

        cn.add(ctc::eval, {v_obs[i][0],p,tx,tv}); // contrainte sur p = x(t) et tv = dtx

    }

    //---------------------------------------------------------------------------------------------------
    // AFFICHAGE AVEC VIBES
    //---------------------------------------------------------------------------------------------------
   
    vibes::beginDrawing();
    VIBesFigMap fig_map2("OFFLINE SLAM");
    fig_map2.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map2.set_properties(100, 100, 600, 300);
    fig_map2.smooth_tube_drawing(true);

    fig_map2.add_tube(&tx, "tx", 0, 1); //Tube avant contraction
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

    for(const auto& b : v_map)
    {
        fig_map2.add_beacon(b, 0.2); // drawing beacons
    }

    vibes::endDrawing();



    //---------------------------------------------------------------------------------------------------
    // ONLINE SLAM
    //---------------------------------------------------------------------------------------------------
    double iteration_dt = 0.2;

    vibes::beginDrawing();
    VIBesFigMap fig_map3("ONLINE SLAM");
    fig_map3.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map3.set_properties(100, 100, 600, 300);
    fig_map3.smooth_tube_drawing(true);

    fig_map3.add_tube(&tx2, "tx", 0, 1); //Tube avant contraction
    fig_map3.add_trajectory(&x_truth, "x*", 0, 1); // tracé de la trajectoire et du robot

    for(const auto& b : v_map)
    {
        fig_map3.add_beacon(b, 0.2); // drawing beacons
    }
    fig_map3.show(0.5);

    // Boites permettant d'estimer la position des landmarks
    vector<IntervalVector> m2(v_map.size(), IntervalVector(2)); //intitilisées à -inf;+inf

    ContractorNetwork cn2;

    double prev_t_obs = tdomain.lb();
    for (double t = tdomain.lb(); t < tdomain.ub(); t += dt)
    {
        if (t - prev_t_obs > 2 * dt) // new observation each 2*dt
        {
            //Création d'une nouvelle observation d'une des landmarks
            Interval &range = cn2.create_dom(Interval());
            j = rand() % v_map.size(); //indice de la landmark choisi aléatoirement
            double dx2 = v_map[j][0] - x_truth(t)[0];
            double dy2 = v_map[j][1] - x_truth(t)[1];
            range = sqrt(sqr(dx2) + sqr(dy2)); //distance mesurée

            //Prise en compte de la nouvelle contrainte liée à l'observation
            IntervalVector &p2 = cn2.create_dom(IntervalVector(3));
            cn2.add(ctc::dist, {m2[j], p2[0], p2[1], range});

            // Contrainte de l'intermériaire p2 = x(t_i) et dx = v
            Interval &t_i = cn2.create_dom(t);
            cn2.add(ctc::eval, {t_i, p2, tx2, tv2});

            // Updated last iteration time
            prev_t_obs = t;
        }

        double contraction_dt = cn2.contract_during(iteration_dt);
        usleep(max(0., iteration_dt - contraction_dt) * 1e6); // pause for the animation

        // Display the current slice [x](t)
        fig_map3.draw_box(tx2(max(0., ibex::previous_float(t))).subvector(0, 1));
    }

    fig_map3.show(0.5);    // argument is robot size

    for (auto &box : m2) // draw identified landmarks
    {
        fig_map3.draw_box(box, "red[]");
    }
        
        
    vibes::endDrawing();
}