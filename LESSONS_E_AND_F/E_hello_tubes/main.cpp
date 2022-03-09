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
    TrajectoryVector actual_x(tdomain, TFunction("(2*cos(t) ; sin(2*t))"), dt); // Trajectoire de Lissajou
    

    IntervalVector b({Interval(0.5), Interval(1)}); //Landmark b

    //Création du tube de actual_x
    TubeVector x(tdomain, dt,TFunction("(2*cos(t); sin(2*t) )"));
    x.inflate(0.2);
    TubeVector x_memo = x; //Pour réinitialiser x afin de montrer les effets des contractions
    cout << "\nE.8. : La trajectoire x∗(⋅) est contenue dans [x](⋅) à chaque instant\n____________________________________________"<<endl;

    //------------------------------------------------------------------------------------

    vibes::beginDrawing();
    VIBesFigMap fig_map("Trajectoire et tube");
    fig_map.set_properties(100, 100, 600, 300);


    fig_map.add_trajectory(&actual_x, "x*", 0, 1); // tracé de la trajectoire et du robot
    fig_map.add_beacon(b, 0.1); // tracé d'une landmark sans passer par .inflate
    fig_map.add_tube(&x, "x", 0, 1);

    fig_map.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map.show(0.5); // argument is robot size;
    

    //------------------------------------------------------------------------------------

    Trajectory actual_y(tdomain, TFunction("sqrt((0.5-2*cos(t))^2 + (1-sin(2*t))^2)"), dt); // Distance entre b et la trajectoire sur tdomain 

    Tube y(actual_y,dt);
    
    VIBesFigTube fig_dist("Distance to the landmark");
    fig_dist.set_properties(100, 100, 600, 300);

    fig_dist.add_trajectory(&actual_y, "y*"); // Tracé de la distance entre b et la trajectoire sur tdomain 
    fig_dist.add_tube(&y, "y");

    fig_dist.show();


    //------------------------------------------------------------------------------------

    tubex::CtcDist C_dist;

    ContractorNetwork cn; // Creating a Contractor Network
    cn.add(C_dist, {x,b,y});
    

    VIBesFigMap fig_map2("Involving tubes in a contractor network");
    fig_map2.set_properties(100, 100, 600, 300);


    fig_map2.add_trajectory(&actual_x, "x*", 0, 1); // tracé de la trajectoire et du robot
    fig_map2.add_beacon(b, 0.1); // tracé d'une landmark sans passer par .inflate
    fig_map2.add_tube(&x, "x", 0, 1);
    fig_map2.show(0.5); // premier affichage du tube

    cn.contract(); //Contraction du tube

    fig_map2.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map2.show(0.5); // Deuxième affichage après contraction
    vibes::endDrawing();

    //------------------------------------------------------------------------------------
    x = x_memo;
    TrajectoryVector actual_v(tdomain, TFunction("(-2*sin(t) ; 2*cos(2*t))"), dt); // Dérivée de la trajectoire

    //Création du tube de actual_v
    TubeVector v(tdomain, dt,TFunction("(-2*sin(t) ; 2*cos(2*t))"));
    v.inflate(0.01);

    tubex::CtcDeriv C_deriv;

    ContractorNetwork cn2; // Creating a Contractor Network
    cn2.add(C_deriv, {x,v});
    cn2.add(C_dist, {x,b,y});
    
    vibes::beginDrawing();
    VIBesFigMap fig_map3("Dealing with differential equations");
    fig_map3.set_properties(100, 100, 600, 300);


    fig_map3.add_trajectory(&actual_x, "x*", 0, 1); // tracé de la trajectoire et du robot
    fig_map3.add_beacon(b, 0.1); // tracé d'une landmark sans passer par .inflate
    fig_map3.add_tube(&x, "x", 0, 1);
    fig_map3.show(0.5); // premier affichage du tube

    cn2.contract(); //Contraction du tube

    fig_map3.axis_limits(-2.5,2.5,-0.1,0.1, true);
    fig_map3.show(0.5); // Deuxième affichage après contraction

    vibes::endDrawing();


    //------------------------------------------------------------------------------------

    RandTrajectory n(tdomain, 0.02, Interval(-0.2,0.2));
    y+=n;

    vibes::beginDrawing();
    VIBesFigTube fig_dist2("Distance to the landmark + noise");
    fig_dist2.set_properties(100, 100, 600, 300);

    //fig_dist2.add_trajectory(&y, "y*"); // Tracé de la distance entre b et la trajectoire sur tdomain 
    fig_dist2.add_tube(&y, "y");

    fig_dist2.show();
    vibes::endDrawing();

}