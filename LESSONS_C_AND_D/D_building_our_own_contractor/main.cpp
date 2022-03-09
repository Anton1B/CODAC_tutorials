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


class MyCtc : public ibex::Ctc
{
public:
    MyCtc(const std::vector<ibex::IntervalVector> &M_)
        : ibex::Ctc(2), // the contractor acts on 2d boxes
          M(M_)         // attribute needed later on for the contraction
    {

    }

    void contract(ibex::IntervalVector &a)
    {
        IntervalVector c = IntervalVector::empty(2);
        //contraction formula here
        for (auto mi : M) //Pour tous les mi appartenant à M
        {
            c |= a & mi; //union des a&mi
        }
        a = c;
    }

protected:
    const std::vector<ibex::IntervalVector> M;
};

int main()
{
  
    vector<IntervalVector> M; //création du set M
    M.push_back(IntervalVector({Interval(1.5), Interval(2.5)}));
    M.push_back(IntervalVector({Interval(3), Interval(1)}));
    M.push_back(IntervalVector(2, Interval(2)));
    M.push_back(IntervalVector({Interval(2.5), Interval(3)}));
    M.push_back(IntervalVector({Interval(3.5), Interval(2)}));
    M.push_back(IntervalVector({Interval(4), Interval(1)}));
    M.push_back(IntervalVector({Interval(1.5), Interval(0.5)}));

    //Ajout des incertitudes
    for(auto& Mi : M)
    {
        Mi.inflate(0.05);
    }

    IntervalVector a1({Interval(1.25, 3), Interval(1.6, 2.75)});
    IntervalVector a2({Interval(2, 3.5), Interval(0.6, 1.2)});
    IntervalVector a3({Interval(1.1, 3.25), Interval(0.2, 1.4)});

    MyCtc ctc_constell(M);
    ContractorNetwork cn;

    cn.add(ctc_constell, {a1});
    cn.add(ctc_constell, {a2});
    cn.add(ctc_constell, {a3});


    // Affichage des valeurs des boites
    cout << "C.3. Test du contracteur :\n";
    cout << a1 << endl;
    cout << a2 << endl;
    cout << a3 << endl;

    vibes::beginDrawing();

    VIBesFigMap fig_map("The constellation constraint and its contractor");
    fig_map.set_properties(100,100,500,500);
    fig_map.axis_limits(0,4,0,4);

    for(auto& Mi : M)
    {
        fig_map.draw_box(Mi, "#FE9A2E[#FE9A2E77]");
    }

    fig_map.draw_box(a1, "blue");
    fig_map.draw_box(a2, "green");
    fig_map.draw_box(a3, "red");
    cn.contract();
    fig_map.draw_box(a1, "#A9A9F5[#A9A9F577]");
    fig_map.draw_box(a2, "#01DF01[#01DF0177]");
    fig_map.draw_box(a3, "#F78181[#F7818177]");

    fig_map.show();
    vibes::endDrawing();
        

    //------------------------------------------------------------------- 

    Vector x_truth({2.,1.,M_PI/6.}); // actual state vector

    vibes::beginDrawing();

    VIBesFigMap fig_map2("Application for localization");
    fig_map2.set_properties(100,100,500,500);
    fig_map2.axis_limits(0,4,0,4);

    //Affichage des landmarks
    for(auto& Mi : M)
    {
        fig_map2.draw_box(Mi, "#FE9A2E[#FE9A2E77]");
    }
    fig_map2.draw_vehicle(x_truth, 0.5);

    //Génération des mesures
    vector<IntervalVector> v_obs =
    DataLoader::generate_static_observations(x_truth, M, false);

    Interval interval_theta;
    Interval interval_rho;

    for(auto& obs : v_obs)
    {
        // obs[0].inflate(0.02); // range
        // obs[1].inflate(0.02); // bearing
        interval_theta = obs[1]+x_truth[2];
        interval_rho = obs[0];
        fig_map2.draw_pie(x_truth[0], x_truth[1], interval_rho, interval_theta, "Gray");
        fig_map2.draw_pie(x_truth[0], x_truth[1], (Interval(0.1)|interval_rho), interval_theta, "lightGray");
    }
    
    fig_map2.show();
    vibes::endDrawing();


    //------------------------------------------------------------------- 

    IntervalVector x{{-oo,oo},{-oo,oo},{M_PI/6.,M_PI/6.}};

    // Déclaration des cotracteurs
    pyibex::CtcPolar ctc_polar;
    CtcFunction ctc_add(Function("d", "m", "x", "d-m+x"));
    CtcFunction ctc_theta(Function("t", "x", "y", "t-x-y"));


    // CONTRACTION
    ContractorNetwork cn2; // Creating a Contractor Network
    for(int i = 0 ; i < v_obs.size() ; i++) // for each measurement
    {
        // Definition of the intermediate variables
        Interval &d1 = cn2.create_dom(Interval());
        Interval &d2 = cn2.create_dom(Interval());
        Interval &theta = cn2.create_dom(Interval());

        cn2.add(ctc_add, {d1, M[i][0], x[0]});
        cn2.add(ctc_add, {d2, M[i][1], x[1]});
        cn2.add(ctc_theta, {theta, x[2], v_obs[i][1]});
        cn2.add(ctc_polar, {d1, d2, v_obs[i][0], theta});

    }
    
    vibes::beginDrawing();

    VIBesFigMap fig_map3("State estimation without data association");
    fig_map3.set_properties(100,100,500,500);
    fig_map3.axis_limits(0,4,0,4);

    //Affichage des landmarks
    for(auto& Mi : M)
    {
        fig_map3.draw_box(Mi, "#FE9A2E[#FE9A2E77]");
    }
    fig_map3.draw_vehicle(x_truth, 0.5);

    for(auto& obs : v_obs)
    {
        obs[0].inflate(0.02); // range
        obs[1].inflate(0.02); // bearing
        interval_theta = obs[1]+x_truth[2];
        interval_rho = obs[0];
        fig_map3.draw_pie(x_truth[0], x_truth[1], interval_rho, interval_theta, "Gray");
        fig_map3.draw_pie(x_truth[0], x_truth[1], (Interval(0.1)|interval_rho), interval_theta, "lightGray");
    }


    cn2.contract();

    fig_map3.draw_box(x.subvector(0,1)); 

    fig_map3.show();
    vibes::endDrawing();


    //------------------------------------------------------------------- 

    x = {{-oo,oo},{-oo,oo},{M_PI/6.,M_PI/6.}};

    // Association set (possible identities)
    vector<IntervalVector> m(v_obs.size(), IntervalVector(2));
    // unknown association for each observation

    MyCtc ctc_asso(M);

    // CONTRACTION
    ContractorNetwork cn3; // Creating a Contractor Network
    for(int i = 0 ; i < v_obs.size() ; i++) // for each measurement
    {
        // Definition of the intermediate variables
        Interval &d1 = cn2.create_dom(Interval());
        Interval &d2 = cn2.create_dom(Interval());
        Interval &theta = cn2.create_dom(Interval());

        cn3.add(ctc_add, {d1, m[i][0], x[0]});
        cn3.add(ctc_add, {d2, m[i][1], x[1]});
        cn3.add(ctc_theta, {theta, x[2], v_obs[i][1]});
        cn3.add(ctc_polar, {d1, d2, v_obs[i][0], theta});

        cn3.add(ctc_asso, {m[i]});

    }
    
    vibes::beginDrawing();

    VIBesFigMap fig_map4("State estimation with data association");
    fig_map4.set_properties(100,100,500,500);
    fig_map4.axis_limits(0,4,0,4);

    //Affichage des landmarks
    for(auto& Mi : M)
    {
        fig_map4.draw_box(Mi, "#FE9A2E[#FE9A2E77]");
    }
    fig_map4.draw_vehicle(x_truth, 0.5);

    for(auto& obs : v_obs)
    {
        interval_theta = obs[1]+x_truth[2];
        interval_rho = obs[0];
        fig_map4.draw_pie(x_truth[0], x_truth[1], interval_rho, interval_theta, "Gray");
        fig_map4.draw_pie(x_truth[0], x_truth[1], (Interval(0.1)|interval_rho), interval_theta, "lightGray");
    }


    cn3.contract();

    //Identification des éléments de M
    for(const auto& mi : m)
    {
        if(mi.max_diam() <= 0.10001) // if identified
        {
            fig_map4.draw_circle(mi[0].mid(), mi[1].mid(), 0.02, "blue[blue]");
        }
    
    }
    


    fig_map4.draw_box(x.subvector(0,1)); 

    fig_map4.show();
    vibes::endDrawing();



}