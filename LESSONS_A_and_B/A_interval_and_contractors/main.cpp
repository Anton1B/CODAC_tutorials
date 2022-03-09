#include <tubex.h>

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
    // Question préliminaire

    // Tube x(Interval(0,10), 0.01, TFunction("cos(t)+abs(t-5)*[-0.1,0.1]"));
    // cout << x << endl;
    // vibes::beginDrawing();
    // VIBesFigTube fig("My first tube");
    // fig.add_tube(&x, "x");
    // fig.show();
    // vibes::endDrawing();

    //--------------------------------------------------------------------------------------
    
    cout << "Exercice A.1\n" << endl;

    cout << "Let us consider two intervals [x]=[8,10] and [y]=[1,2]." << endl;
    cout << "[x]/[y] = " << "[4,10]" << endl;
    cout << "\n-----------------------------------------------"<<endl;

    //--------------------------------------------------------------------------------------
    
    cout << "Exercice A.2\n" << endl;

    Interval x(-2,4);
    Interval y(1,3);
    cout << x << "." << y << " = " << x*y << endl;

    x = Interval(8,10);
    y = Interval(-1,0);
    cout << x << "/" << y << " = " << x/y << endl;

    x = Interval(-2,4);
    y = Interval(6,7);
    cout << x << "U" << y << " = " << (x|y) << endl;
    
    x = Interval(2,7);
    y = Interval(1,9);
    cout <<"max(" << x << "," << y << ")" << " = " << max(x,y) << endl;

    x = Interval::EMPTY_SET;
    y = Interval(1,9);
    cout <<"max(" << x << "," << y << ")" << " = " << max(x,y) << endl;


    x = Interval(-oo,oo);
    cout <<"cos(" << x << ")" << " = " << cos(x) << endl;

    x = Interval(-1,4);
    cout <<"sqr(" << x << ")" << " = " << sqr(x) << endl;
    
    x = Interval(1,2);
    y = Interval(-1,3);
    Interval a(-2,4);
    Interval b(6,7);
    cout <<"(" << x << "." << y << ")" << " + " << "max" <<"( "<< a << "&"<< b << " , " << a << ")" << " = " << ((x*y)+ max((a&b),a)) << endl;
    
    cout << "\n-----------------------------------------------"<<endl;

    //--------------------------------------------------------------------------------------

    cout << "Exercice A.3\n" << endl;

    x = Interval(0,M_PI);
    y = Interval(-M_PI/6,M_PI/6);
    cout << "[0,π].[−π/6,π/6]" << " = " << x*y << endl;

    cout << "\n-----------------------------------------------"<<endl;

    //--------------------------------------------------------------------------------------

    cout << "Exercice A.4\n" << endl;

    IntervalVector vx(2, Interval(0,0)); 
    IntervalVector vb{{3,4},{2,3}}; 
    IntervalVector c = cart_prod(vx,vb);



    Function g("c[4]", "sqrt( (c[0]-c[2])^2 + (c[1]-c[3])^2 )");
    cout << "g([x],[b]) = g([c]) = [d] = " << g.eval(c) << endl;

    cout << "\n-----------------------------------------------"<<endl;

    //--------------------------------------------------------------------------------------

    cout << "Exercice A.6\n" << endl;
    cout << "boxes [x] and [b]" << endl;
    vibes::beginDrawing();
    VIBesFigMap fig("Map");
    fig.set_properties(50, 50, 400, 400); // position and size

    cout << "Ring containing the set of all positions that are d-distant from x=(0,0), with d∈[d]." << endl;
    fig.draw_box(vx);
    fig.draw_box(vb);

    Interval d = g.eval(c);
    fig.draw_circle(0.,0.,d.lb());
    fig.draw_circle(0.,0.,d.ub());

    fig.show(); // display all items of the figure
    vibes::endDrawing();

    cout << "\n-----------------------------------------------"<<endl;

    //--------------------------------------------------------------------------------------

    cout << "Exercice A.7\n" << endl;

    vx.inflate(0.1);
    cout << "[x] = " << vx << endl;

    vibes::beginDrawing();
    VIBesFigMap fig2("Map 2");
    fig2.set_properties(50, 50, 400, 400); // position and size

    fig2.draw_box(vx,"red[yellow]");
    fig2.draw_box(vb);
    fig2.draw_box(vb+vx,"red");

    c = cart_prod(vx,vb);
    d = g.eval(c);
    fig2.draw_circle(0.,0.,d.lb());
    fig2.draw_circle(0.,0.,d.ub());

    fig2.show(); // display all items of the figure
    vibes::endDrawing();

    cout << "Result reliable according to the sets [x] and [b] " << endl;
    cout << "\n-----------------------------------------------"<<endl;

    //--------------------------------------------------------------------------------------

    cout << "Exercice A.8\n" << endl;
    Function f("x[2]","b[2]","d", "sqrt( (x[0]-b[0])^2 + (x[1]-b[1])^2 ) - d");
    CtcFunction C_dist(f);
    cout << "Definition of a contractor Cdist related to the distance constraint between two 2d positions x and b∈R^2." << endl;
  
    cout << "\n-----------------------------------------------"<<endl;

    //--------------------------------------------------------------------------------------

    cout << "Exercice A.9\n" << endl;

    vx = {{0.,0.},{0.,0.}}; 
    IntervalVector vb1{{1.5,2.5},{4,11}}; 
    IntervalVector vb2{{3,4},{4,6.5}}; 
    IntervalVector vb3{{5,7},{5.5,8}}; 
    d = Interval(7,8);

    vibes::beginDrawing();
    VIBesFigMap fig3("Map 3");
    fig3.set_properties(50, 50, 400, 400); // position and size

    fig3.draw_box(vx);
    fig3.draw_box(vb1,"blue");
    fig3.draw_box(vb2,"blue");
    fig3.draw_box(vb3,"blue");

    fig3.draw_circle(0.,0.,d.lb());
    fig3.draw_circle(0.,0.,d.ub());

    // CONTRACTION
    ContractorNetwork cn;       // Creating a Contractor Network
    cn.add(C_dist, {vx,vb1,d}); // Adding the C+ contractor to the network,
                            // applied on three domains listed between braces
    cn.add(C_dist, {vx,vb2,d});
    cn.add(C_dist, {vx,vb3,d});
    cn.contract();

    fig3.draw_box(vb1,"blue[blue]");
    fig3.draw_box(vb2,"blue[blue]");
    fig3.draw_box(vb3,"blue[blue]");

    fig3.draw_circle(0.,0.,d.lb());
    fig3.draw_circle(0.,0.,d.ub());

    fig3.show(); // display all items of the figure
    vibes::endDrawing();

    cout << "Drawing of the boxes and the ring before and after the contractions"

    






}
