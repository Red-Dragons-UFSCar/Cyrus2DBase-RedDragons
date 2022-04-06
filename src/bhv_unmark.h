//
// Created by nader on 2022-04-06.
//

#ifndef CYRUS2DBASE_BHV_UNMARK_H
#define CYRUS2DBASE_BHV_UNMARK_H
#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>
#include <rcsc/player/abstract_player_object.h>
#include <vector>
using namespace std;
using namespace rcsc;
class Bhv_Unmark
        : public rcsc::SoccerBehavior {
public:
    static int last_cycle ;
    static Vector2D target_pos;
    Bhv_Unmark(){}
    enum posType{
        leadPos,
        throuPos,
        simplePos
    };
    struct passpos
    {
        Vector2D target;
        double ballspeed;
        bool conf;
        double eval;
        double cycleintercept;
        passpos(Vector2D t=Vector2D(100,100),double v=0,bool c=false,double ev=0,double cyc=0)
                :target(t),ballspeed(v),conf(c),eval(ev),cycleintercept(cyc)
        {}
    };
    struct UnmarkPosition
    {
        Vector2D ballpos;
        int holder;
        Vector2D target;
        int cycle2target;
        posType type;
        bool conf;
        bool tmconf;
        double eval;
        vector<passpos> passposvector;
        UnmarkPosition(Vector2D bp=Vector2D(100,100),int tm=0,Vector2D tar=Vector2D(100,100),int d2t=0,posType pt=leadPos,bool c=false,bool tmc=false,
                 double ev=0,vector<passpos> ppv=vector<passpos>())
                :ballpos(bp),holder(tm),target(tar),cycle2target(d2t),type(pt),conf(c),tmconf(tmc),eval(ev),passposvector(ppv)
        {}
    };

    bool execute(PlayerAgent * agent);
    bool canIpos(PlayerAgent * agent);
    int passer_finder(PlayerAgent *agent);
    void simulate_dash(PlayerAgent * agent , int passer ,std::vector<Bhv_Unmark::UnmarkPosition> & unmark_positions);
    double nearest_tm_dist_to(const WorldModel & wm,
                                Vector2D point);
    void lead_pass_simulator(const WorldModel & wm,
                               Vector2D passer_pos,
                               Vector2D new_self_pos,
                               int n_step,
                               vector<passpos> & passes);
    int self_cycle_intercept(const WorldModel & wm,
                             Vector2D pass_start,
                             double pass_speed,
                             Vector2D & pass_target,
                             double self_body,
                             double speed);
    int opponents_cycle_intercept(const WorldModel & wm,
                                     Vector2D pass_start,
                                     double pass_speed,
                                     int pass_cycle,
                                     Vector2D pass_target);
    int opponent_cycle_intercept(const WorldModel & wm,
                                    const rcsc::AbstractPlayerObject * opp,
                                    Vector2D pass_start,
                                    double pass_speed,
                                    int pass_cycle,
                                    Vector2D pass_target);
    double evaluate_position(const WorldModel & wm, UnmarkPosition pos);
    bool run(PlayerAgent * agent,UnmarkPosition Pos);
};
#endif //CYRUS2DBASE_BHV_UNMARK_H
