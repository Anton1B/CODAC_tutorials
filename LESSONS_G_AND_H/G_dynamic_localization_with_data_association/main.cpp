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
    Interval tdomain(0,6); // temporal domain [t0,tf]

    //------------------------------------------------------------------------------------------------------------
    
    //Création de la trajectoire
    TrajectoryVector actual_x(tdomain, TFunction("(10*cos(t)+t ; 5*sin(2*t)+t)"), dt); // Trajectoire de Lissajou

    //Création du tube x sans initialisation
    TubeVector x(tdomain, dt, 2); // initialization with [-∞,∞]×[-∞,∞]

    //------------------------------------------------------------------------------------------------------------
    
    // Creating random map of landmarks
    int nb_landmarks = 150;
    IntervalVector map_area(actual_x.codomain().subvector(0,1));
    map_area.inflate(2);
    vector<IntervalVector> v_map = DataLoader::generate_landmarks_boxes(map_area, nb_landmarks);



    // Generating observations obs=(t,range,bearing) of these landmarks
    int max_nb_obs = 20;
    Interval visi_range(0,4); // [0m,75m]
    Interval visi_angle(-M_PI/4,M_PI/4); // frontal sonar

    vector<IntervalVector> v_obs = DataLoader::generate_observations(actual_x, v_map, max_nb_obs,true, visi_range, visi_angle);

    //inflation
    for(auto& obs : v_obs)
    {
        obs[1].inflate(0.1); // range
        obs[2].inflate(0.04); // bearing
    }

    //------------------------------------------------------------------------------------------------------------
    
    vibes::beginDrawing();
    VIBesFigMap fig_map("range-and-bearing measurements with uncertainties");
    fig_map.set_properties(100, 100, 600, 300);

    fig_map.add_trajectory(&actual_x, "x*", 0, 1); // tracé de la trajectoire et du robot
    fig_map.add_observations(v_obs, &actual_x); // drawing obs
    
    for(const auto& b : v_map)
    {
        fig_map.add_beacon(b.mid(), 0.1); // drawing beacons
    }

    fig_map.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map.show(0.5); // Deuxième affichage après contraction

    vibes::endDrawing();


    //------------------------------------------------------------------------------------------------------------
    // DYNAMIC RANGE ONLY LOCALIZATION
    //------------------------------------------------------------------------------------------------------------

    Interval tdomain2(0,6); // temporal domain [t0,tf]


    TrajectoryVector etat_x(tdomain2, TFunction("(10*cos(t)+t ; 5*sin(2*t)+t ; atan2((10*cos(2*t)+1),(-10*sin(t)+1)) ; sqrt(sqr(-10*sin(t)+1)+sqr(10*cos(2*t)+1)))"), dt); //Trajectoire de l'état x

    //Initialisation du tube de l'état x
    TubeVector tx({Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, TFunction("atan2((10*cos(2*t)+1),(-10*sin(t)+1))")), Tube(tdomain2, dt, TFunction("sqrt(sqr(-10*sin(t)+1)+sqr(10*cos(2*t)+1))"))});

    //Incertitude des mesures
    tx[2].inflate(0.01);
    tx[3].inflate(0.01);

    //Initialisation de la dérivée du tube de l'état x
    TubeVector tdx({Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, Interval())});

    //---------------------------------------------------------------------------------------------------
    // PREPARATION DES CONTRAINTES
    //---------------------------------------------------------------------------------------------------

    // Création de contracteurs

    CtcFunction ctc_add(Function("d", "m", "x", "d-m+x"));
    CtcFunction ctc_theta(Function("t", "x", "y", "t-x-y"));

    // Equations d'état 
    CtcFunction ctc_dx1(Function("psi","v","dx","v*cos(psi)-dx"));
    CtcFunction ctc_dx2(Function("psi","v","dx","v*sin(psi)-dx"));

    vector<IntervalVector> m(v_obs.size(), IntervalVector(2));

    CtcConstell ctc_constell(v_map);

    //---------------------------------------------------------------------------------------------------
    // RESEAU DE CONTRACTIONS
    //---------------------------------------------------------------------------------------------------

    ContractorNetwork cn2; //Création du réseau de contracteurs
  
    // contrainte sur yi --> yi = gi(p)
    for (int i=0;i<v_obs.size();i++)
    {   

        Interval &d1 = cn2.create_dom(Interval());
        Interval &d2 = cn2.create_dom(Interval());
        Interval &psi = cn2.create_dom(Interval());

        IntervalVector &p = cn2.create_dom(IntervalVector(4)); // Initialisation des intermédiaires p

        cn2.add(ctc_add, {d1, m[i][0], p[0]});
        cn2.add(ctc_add, {d2, m[i][1], p[1]});
        cn2.add(ctc_theta, {psi, p[2], v_obs[i][2]});
        cn2.add(ctc::polar, {d1, d2, v_obs[i][1], psi});

        cn2.add(ctc::eval, {v_obs[i][0],p,tx,tdx}); // évaluation de p = x(ti)

        cn2.add(ctc_constell,{m[i]});

    }

    // contrainte sur dx1 et dx2 en fonction de psi et v mesurées
    cn2.add(ctc_dx1, {tx[2],tx[3], tdx[0]});
    cn2.add(ctc_dx2, {tx[2],tx[3], tdx[1]});

    //---------------------------------------------------------------------------------------------------
    // AFFICHAGE AVEC VIBES
    //---------------------------------------------------------------------------------------------------
   
    vibes::beginDrawing();

    VIBesFigMap fig_map4("Localization by solving data association");
    fig_map4.set_properties(100, 100, 600, 300);
    fig_map4.smooth_tube_drawing(true);

    fig_map4.add_tube(&tx, "tx", 0, 1); //Tube avant contraction

    fig_map4.add_trajectory(&etat_x, "x*", 0, 1); // tracé de la trajectoire et du robot

    //Tracé des landmarks
    fig_map4.add_observations(v_obs, &etat_x); // drawing obs
    
    for(const auto& b : v_map)
    {
        fig_map4.add_beacon(b.mid(), 0.1); // drawing beacons
    }

    fig_map4.axis_limits(-2.5,2.5,-0.1,0.1, true);

    cn2.contract();
    fig_map4.show(0.5);

    vibes::endDrawing();

}