//
// Created by nader on 2022-04-06.
//

#ifndef CYRUS2DBASE_BHV_UNMARK_H
#define CYRUS2DBASE_BHV_UNMARK_H

#include <rcsc/geom/vector_2d.h>
#include <rcsc/player/soccer_action.h>
#include <rcsc/player/abstract_player_object.h>
#include <utility>
#include <vector>

using namespace std;
using namespace rcsc;

class Bhv_Unmark
        : public rcsc::SoccerBehavior {
public:
    Bhv_Unmark() = default;

    struct UnmakingPass {
        Vector2D pass_target;
        double pass_speed;
        double pass_eval;
        double pass_cycle;

        explicit UnmakingPass(Vector2D target, double speed, double eval, double cycle)
                : pass_target(target), pass_speed(speed), pass_eval(eval), pass_cycle(cycle) {
        }
    };

    struct UnmarkPosition {
        int id;
        Vector2D ball_pos;
        Vector2D target;
        double eval;
        vector<UnmakingPass> pass_list;
        int end_valid_cycle;
        int last_run_cycle;
        UnmarkPosition(int id, Vector2D ball_pos, Vector2D target, double eval, vector<UnmakingPass> pass_list)
                : id(id), ball_pos(ball_pos), target(target), eval(eval), pass_list(std::move(pass_list)),
                  end_valid_cycle(0), last_run_cycle(0) {
        }
        UnmarkPosition(){
            target = Vector2D::INVALIDATED;
        }
    };

    static UnmarkPosition last_unmark_position;

    bool execute(PlayerAgent *agent) override;

    bool can_unmarking(const rcsc::WorldModel &wm);

    int passer_finder(PlayerAgent *agent);

    void simulate_dash(PlayerAgent *agent, int passer, std::vector<Bhv_Unmark::UnmarkPosition> &unmark_positions);

    double nearest_tm_dist_to(const WorldModel &wm,
                              Vector2D point);

    void lead_pass_simulator(const WorldModel &wm,
                             Vector2D passer_pos,
                             Vector2D unmark_target,
                             int n_step,
                             vector<UnmakingPass> &passes);

    int pass_travel_cycle(Vector2D pass_start, double pass_speed, Vector2D &pass_target);

    int opponents_cycle_intercept(const WorldModel &wm,
                                  Vector2D pass_start,
                                  double pass_speed,
                                  Vector2D pass_target);

    int opponent_cycle_intercept(const AbstractPlayerObject *opp, Vector2D pass_start, double pass_speed,
                                 Vector2D pass_target);

    double evaluate_position(const WorldModel &wm, const UnmarkPosition &unmark_position);

    bool run(PlayerAgent *agent, const UnmarkPosition &unmark_position);
};

#endif //CYRUS2DBASE_BHV_UNMARK_H
