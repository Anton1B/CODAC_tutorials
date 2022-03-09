#include <tubex.h>
#include <math.h>
#include <vector>
#include <unistd.h>
#include <tubex-pyibex.h>

using namespace std;
using namespace tubex;

void onlineSLAM()
{
    //H.1. Init temporal domain
    double dt = 0.05;
    Interval tdomain(0, 15);
    double iteration_dt = 0.2; // elapsed animation time between each dt

    // Initial pose x0=(0,0,2) is known which enables to contract the first box
    Vector x0({0, 0, 2});

    // System input
    Trajectory u(tdomain, TFunction("3*(sin(t)^2)+t/100"), dt);
    RandTrajectory n_u(tdomain, dt, Interval(-0.03, 0.03));  //noise
    RandTrajectory n_x3(tdomain, dt, Interval(-0.03, 0.03)); //noise

    // Actual trajectories (state + derivative)
    TrajectoryVector v_truth(3);
    TrajectoryVector x_truth(3);
    v_truth[2] = u + n_u;
    x_truth[2] = v_truth[2].primitive() + x0[2] + n_x3;
    v_truth[0] = 10 * cos(x_truth[2]);
    v_truth[1] = 10 * sin(x_truth[2]);
    x_truth[0] = v_truth[0].primitive() + x0[0];
    x_truth[1] = v_truth[1].primitive() + x0[1];

    // H.2. Init tubes
    TubeVector x(3, Tube(tdomain, dt, Interval()));
    TubeVector v(3, Tube(tdomain, dt, Interval()));
    v[2] = Tube(u + n_u, dt).inflate(0.03);
    x[2] = Tube(x_truth[2], dt).inflate(0.03); // we suppose that the heading x3 is perfectly known, thus equal to x3*
    v[0] = 10 * cos(x[2]);
    v[1] = 10 * sin(x[2]);
    x[0] = v[0].primitive() + x0[0];
    x[1] = v[1].primitive() + x0[1];

    //H.3 landmarks coordinates
    vector<Vector> v_beacons({{6, 12},
                              {-2, -5},
                              {-3, 20},
                              {3, 4}});

    vibes::beginDrawing();
    VIBesFigMap fig_map("Map");
    fig_map.set_properties(100, 100, 600, 300);
    fig_map.add_trajectory(&x_truth, "x*", 0, 1, "white");
    fig_map.axis_limits(-2.5, 2.5, -0.1, 0.1, true);
    fig_map.add_tube(&x, "x", 0, 1);
    fig_map.smooth_tube_drawing(true);
    for (const auto &b : v_beacons)
        fig_map.add_beacon(b, 0.5); // drawing beacons
    fig_map.show(1);                // argument is robot size

    ContractorNetwork cn;

    //associated landmark coordinates of v_beacons to each measurement of v_obs
    vector<IntervalVector> M(v_beacons.size(), IntervalVector(2)); // coordinates x and y of the landmark observed initialized at infinite

    // Create tubes defined over [t0,tf]
    // Add already known constraints, such as motion equations

    double prev_t_obs = tdomain.lb();
    for (double t = tdomain.lb(); t < tdomain.ub(); t += dt)
    {
        if (t - prev_t_obs > 2 * dt) // new observation each 2*dt
        {
            // Creating new observation to a random landmark
            Interval &range = cn.create_dom(Interval());
            int i = rand() % v_beacons.size(); //index of the considered beacon
            double dx = v_beacons[i][0] - x_truth(t)[0];
            double dy = v_beacons[i][1] - x_truth(t)[1];
            range = sqrt(sqr(dx) + sqr(dy)); //distance to the beacon

            // Adding related observation constraints to the network
            IntervalVector &p = cn.create_dom(IntervalVector(3));
            cn.add(ctc::dist, {M[i], p[0], p[1], range});

            // constraint p = x(ti) and dx = v
            Interval &ti = cn.create_dom(t);
            cn.add(ctc::eval, {ti, p, x, v});

            // Updated last iteration time
            prev_t_obs = t;
        }

        double contraction_dt = cn.contract_during(iteration_dt);
        usleep(max(0., iteration_dt - contraction_dt) * 1e6); // pause for the animation

        // Display the current slice [x](t)
        fig_map.draw_box(x(max(0., ibex::previous_float(t))).subvector(0, 1));
    }
    fig_map.show(1);    // argument is robot size
    for (auto &box : M) // draw identified landmarks
        fig_map.draw_box(box, "red[]");
    vibes::endDrawing();
}

void offlineSLAM()
{
    //H.1. Init temporal domain
    double dt = 0.05;
    Interval tdomain(0, 15);

    // Initial pose x0=(0,0,2) is known which enables to contract the first box
    Vector x0({0, 0, 2});

    // System input
    Trajectory u(tdomain, TFunction("3*(sin(t)^2)+t/100"), dt);
    RandTrajectory n_u(tdomain, dt, Interval(-0.03, 0.03));  //noise
    RandTrajectory n_x3(tdomain, dt, Interval(-0.03, 0.03)); //noise

    // Actual trajectories (state + derivative)
    TrajectoryVector v_truth(3);
    TrajectoryVector x_truth(3);
    v_truth[2] = u + n_u;
    x_truth[2] = v_truth[2].primitive() + x0[2] + n_x3;
    v_truth[0] = 10 * cos(x_truth[2]);
    v_truth[1] = 10 * sin(x_truth[2]);
    x_truth[0] = v_truth[0].primitive() + x0[0];
    x_truth[1] = v_truth[1].primitive() + x0[1];

    // H.2. Init tubes
    TubeVector x(3, Tube(tdomain, dt, Interval()));
    TubeVector v(3, Tube(tdomain, dt, Interval()));
    v[2] = Tube(u + n_u, dt).inflate(0.03);
    x[2] = Tube(x_truth[2], dt).inflate(0.03); // we suppose that the heading x3 is perfectly known, thus equal to x3*
    v[0] = 10 * cos(x[2]);
    v[1] = 10 * sin(x[2]);
    x[0] = v[0].primitive() + x0[0];
    x[1] = v[1].primitive() + x0[1];

    //H.3 landmarks coordinates
    vector<Vector> v_beacons({{6, 12},
                              {-2, -5},
                              {-3, 20},
                              {3, 4}});

    vector<IntervalVector> v_obs;
    // Generate observations every 2*dt
    for (float t = tdomain.lb(); t <= tdomain.ub(); t += 2 * dt)
    {
        int i = rand() % v_beacons.size();
        double dx = v_beacons[i][0] - x_truth(t)[0];
        double dy = v_beacons[i][1] - x_truth(t)[1];
        Interval range = sqrt(sqr(dx) + sqr(dy));
        v_obs.push_back(IntervalVector({Interval(t), range.inflate(0.03), Interval(i)})); // instant t of the measurement, landmark i observed, range, bearing
    }

    vibes::beginDrawing();
    VIBesFigMap fig_map("Map");
    fig_map.set_properties(100, 100, 600, 300);
    fig_map.add_trajectory(&x_truth, "x*", 0, 1, "white");
    fig_map.axis_limits(-2.5, 2.5, -0.1, 0.1, true);
    fig_map.add_tube(&x, "x", 0, 1);
    fig_map.smooth_tube_drawing(true);
    for (const auto &b : v_beacons)
        fig_map.add_beacon(b, 0.5); // drawing beacons
    fig_map.show(1);                // argument is robot size

    ContractorNetwork cn;

    //associated landmark coordinates of v_beacons to each measurement of v_obs
    vector<IntervalVector> M(v_beacons.size(), IntervalVector(2)); // coordinates x and y of the landmark observed initialized at infinite

    for (int i = 0; i < v_obs.size(); i++) // for each measurement
    {
        //constraints about the distances to the landmarks for each measure yi : yi = gi(p)
        // Add contractors and related domains
        IntervalVector &p = cn.create_dom(IntervalVector(3));
        int j = v_obs[i][2].mid(); // associated landmark
        cn.add(ctc::dist, {M[j], p[0], p[1], v_obs[i][1]});

        // constraint p = x(ti) and dx = v
        cn.add(ctc::eval, {v_obs[i][0], p, x, v});
    }

    cn.contract();

    fig_map.show(1);    // argument is robot size
    for (auto &box : M) // draw identified landmarks
        fig_map.draw_box(box, "red[]");
    vibes::endDrawing();
}

int main()
{
    // offlineSLAM();
    onlineSLAM();
    return EXIT_SUCCESS;
}