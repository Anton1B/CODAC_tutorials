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
    //Création de la trajectoire
    double dt = 0.01;
    Interval tdomain(0,5); // temporal domain [t0,tf]
    Interval t(2.0,2.0);

    IntervalVector y({{-0.84,-0.83},{-0.76,-0.75}}); //Landmark observation y

    //------------------------------------------------------------------------------------------------------------

    TrajectoryVector actual_x(tdomain, TFunction("(2*cos(t) ; sin(2*t))"), dt); // Trajectoire de Lissajou

    //Création du tube x sans initialisation
    TubeVector x(tdomain, dt, 2); // initialization with [-∞,∞]×[-∞,∞]

    //------------------------------------------------------------------------------------------------------------

    TrajectoryVector actual_v(tdomain, TFunction("(-2*sin(t) ; 2*cos(2*t))"), dt); // Dérivée de la trajectoire

    //Création du tube de actual_v
    TubeVector v(tdomain, dt,TFunction("(-2*sin(t) ; 2*cos(2*t))"));
    v.inflate(0.03);

    //------------------------------------------------------------------------------------------------------------

    tubex::CtcEval C_eval;

    ContractorNetwork cn; // Creating a Contractor Network
    cn.add(C_eval, {t,y,x,v});
    cn.contract(); //Contraction du tube
    
    vibes::beginDrawing();
    VIBesFigMap fig_map3("Contraction of the tube at t=2.0");
    fig_map3.set_properties(100, 100, 600, 300);

    fig_map3.add_trajectory(&actual_x, "x*", 0, 1); // tracé de la trajectoire et du robot
    fig_map3.add_beacon(y, 0.1); // tracé d'une landmark sans passer par .inflate
    fig_map3.add_tube(&x, "x", 0, 1);

    fig_map3.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map3.show(0.5); // Deuxième affichage après contraction

    vibes::endDrawing();


    //------------------------------------------------------------------------------------------------------------
    // DYNAMIC RANGE ONLY LOCALIZATION
    //------------------------------------------------------------------------------------------------------------

    Interval tdomain2(0,3); // temporal domain [t0,tf]

    //Time ti
    double t1 = 0.3;
    double t2 = 1.5;
    double t3 = 2;

    //Landmark  bi
    Vector b1({8, 3});
    Vector b2({0, 5});
    Vector b3({-2, 1});

    //Distances yi
    Interval y1(1.8,1.8);
    Interval y2(3.6,3.6);
    Interval y3(2.8,2.8);
    y1.inflate(0.1);
    y2.inflate(0.1);
    y3.inflate(0.1);


   TrajectoryVector etat_x(tdomain2, TFunction("(10*cos(t)+t ; 5*sin(2*t)+t ; atan2((10*cos(2*t)+1),(-10*sin(t)+1)) ; sqrt(sqr(-10*sin(t)+1)+sqr(10*cos(2*t)+1)))"), dt); //Trajectoire de l'état x

   //Initialisation du tube de l'état x
    TubeVector tx(etat_x, dt);

    //Incertitudes
    tx[0].inflate(0.5); 
    tx[1].inflate(0.5); 

    tx[2].inflate(0.01); 
    tx[3].inflate(0.01); 

    //Initialisation de la dérivée du tube de l'état x
    TubeVector tdx({Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, Interval())});

    //---------------------------------------------------------------------------------------------------
    // PREPARATION DES CONTRAINTES
    //---------------------------------------------------------------------------------------------------

    // Initialisation des placements pi par rapport aux observations des distances aux landmarks
    IntervalVector p1 = IntervalVector(2);
    IntervalVector p2 = IntervalVector(2);
    IntervalVector p3 = IntervalVector(2);
    
    IntervalVector obs_b1(b1);
    IntervalVector obs_b2(b2);
    IntervalVector obs_b3(b3);

    //---------------------------------------------------------------------------------------------------
    TubeVector dx1_dx2({Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, Interval())});
    TubeVector x1_x2({Tube(tdomain2, dt, Interval()), Tube(tdomain2, dt, Interval())});
    CtcFunction ctc_add(Function("x", "x2", "x-x2")); // pi - x(ti) = 0

    //---------------------------------------------------------------------------------------------------
    // RESEAU DE CONTRACTIONS
    //---------------------------------------------------------------------------------------------------

    ContractorNetwork cn2; //Création du réseau de contracteurs

    // contrainte sur yi --> yi = gi(p)
    cn2.add(ctc::dist, {p1, obs_b1, y1});
    cn2.add(ctc::dist, {p2, obs_b2, y2});
    cn2.add(ctc::dist, {p3, obs_b3, y3});

    // contrainte sur pi --> pi = x(ti)
    cn2.add(ctc_add, {tdx[0], dx1_dx2[0]});
    cn2.add(ctc_add, {tdx[1], dx1_dx2[1]});
    cn2.add(ctc_add, {tx[0], x1_x2[0]});
    cn2.add(ctc_add, {tx[1], x1_x2[1]});

    // Contraction du tube à des moments donnés
    cn2.add(ctc::eval, {t1, p1, x1_x2, dx1_dx2});
    cn2.add(ctc::eval, {t2, p2, x1_x2, dx1_dx2});
    cn2.add(ctc::eval, {t3, p3, x1_x2, dx1_dx2});

    //Contrainte liant x et dx
    cn2.add(ctc::deriv, {tx, tdx});

    //Contrainte de la fonction d'évolution f --> dx = f(x)
    cn2.add(ctc::polar, {tdx[0], tdx[1], tx[3], tx[2]}); //On exprime (v*cos(ψ) ; v*sin(ψ)) avec rho = v = tx[3] et theta = psi = tx[2]


    //---------------------------------------------------------------------------------------------------
    // AFFICHAGE AVEC VIBES
    //---------------------------------------------------------------------------------------------------
   
    vibes::beginDrawing();

    VIBesFigMap fig_map4("3 contractions of the tube");
    fig_map4.set_properties(100, 100, 600, 300);

    fig_map4.add_trajectory(&etat_x, "x*", 0, 1); // tracé de la trajectoire et du robot

    //Tracé des landmarks
    fig_map4.add_beacon(b1, 0.15);
    fig_map4.add_beacon(b2, 0.15);
    fig_map4.add_beacon(b3, 0.15);

    fig_map4.add_tube(&tx, "tx", 0, 1); //Tube avant contraction

    fig_map4.axis_limits(-2.5,2.5,-0.1,0.1, true);

    fig_map4.show(0.5); // Premier affichage avant contraction
    //-------------------------------------------------------------

    cn2.contract();

    //Tracé des anneaux
    fig_map4.draw_ring(b1[0], b1[1], y1.lb(), "orange");
    fig_map4.draw_ring(b1[0], b1[1], y1.ub(), "orange");

    fig_map4.draw_ring(b2[0], b2[1], y2.lb(), "orange");
    fig_map4.draw_ring(b2[0], b2[1], y2.ub(), "orange");

    fig_map4.draw_ring(b3[0], b3[1], y3.lb(), "orange");
    fig_map4.draw_ring(b3[0], b3[1], y3.ub(), "orange");

    //Création des états du robot à ti : (x,y,psi)
    Vector t1_p({p1[0].mid(), p1[1].mid(), etat_x(t1)[2]});  
    Vector t2_p({p2[0].mid(),p2[1].mid(), etat_x(t2)[2]}); 
    Vector t3_p({p3[0].mid(),p3[1].mid(), etat_x(t3)[2]}); 

    //Tracé du robot
    fig_map4.draw_vehicle(t1_p, 0.7);
    fig_map4.draw_vehicle(t2_p, 0.7); 
    fig_map4.draw_vehicle(t3_p, 0.7);  

    fig_map4.show(0.5); // Deuxième affichage après contraction (passage du tube non contracté en gris)

    vibes::endDrawing();

}