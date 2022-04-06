//
// Created by nader on 2022-04-06.
//

#ifndef CYRUS2DBASE_BHV_UNMARK_H
#define CYRUS2DBASE_BHV_UNMARK_H
#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>
#include <vector>
using namespace std;
using namespace rcsc;
class bhv_unmark /*: public rcsc::SoccerBehavior */{
public:
    static int last_cycle ;
    static Vector2D target_pos;
    bhv_unmark(){}
    enum posType{
        leadPos,
        throuPos,
        simplePos
    };
    struct posfor
    {
        int firstUnum;
        posType firstType;
        double firstmaxD2pos;
        double firstmaxD2tar;

        posfor(int fu=0,posType ft=leadPos , double fd2p=0 , double fd2t=0)
                :firstUnum(fu),firstType(ft),firstmaxD2pos(fd2p),firstmaxD2tar(fd2t)
        {}
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
    struct Position
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
        Position(Vector2D bp=Vector2D(100,100),int tm=0,Vector2D tar=Vector2D(100,100),int d2t=0,posType pt=leadPos,bool c=false,bool tmc=false,
                 double ev=0,vector<passpos> ppv=vector<passpos>())
                :ballpos(bp),holder(tm),target(tar),cycle2target(d2t),type(pt),conf(c),tmconf(tmc),eval(ev),passposvector(ppv)
        {}
    };

    bool execute(PlayerAgent * agent);
    bool canIpos(PlayerAgent * agent);
    posfor whatTm(PlayerAgent *agent);
    void simulate_dash(PlayerAgent * agent , int tm ,double maxD2pos,double maxD2tar,std::vector<bhv_unmark::Position> & posPosition);
    int predict_player_turn_cycle( const rcsc::PlayerType * ptype,
                                   const AngleDeg & player_body,
                                   const double & player_speed,
                                   const double & target_dist,
                                   const AngleDeg & target_angle,
                                   const double & dist_thr,
                                   const bool use_back_dash);
    double dist_near_tm(const WorldModel & wm,
                        Vector2D point);
    void lead_pass_simulat(const WorldModel & wm,
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
    int opps_cycle_intercept(const WorldModel & wm,
                             Vector2D pass_start,
                             double pass_speed,
                             int pass_cycle,
                             Vector2D pass_target);
    int opp_cycle_intercept(const WorldModel & wm,
                            AbstractPlayerObject * opp,
                            Vector2D pass_start,
                            double pass_speed,
                            int pass_cycle,
                            Vector2D pass_target);
    double posEval(const WorldModel & wm,
                   Position pos , rcsc::PlayerAgent * agent);
    bool run(PlayerAgent * agent,Position Pos);
};
#endif //CYRUS2DBASE_BHV_UNMARK_H
