/*
 * bhv_unmark.cpp
 *
 *  Created on: Nov 12, 2012
 *      Author: 007
 */
#ifdef HAVE_CONFIG_H

#include <config.h>

#endif

#include "strategy.h"
#include "bhv_unmark.h"
#include "intention_receive.h"
#include "chain_action/field_analyzer.h"
#include <vector>

#include <rcsc/player/say_message_builder.h>
#include <rcsc/action/basic_actions.h>
#include <rcsc/action/body_go_to_point.h>
#include <rcsc/action/neck_turn_to_ball_or_scan.h>
#include <rcsc/math_util.h>
#include <rcsc/player/player_agent.h>
#include <rcsc/player/intercept_table.h>
#include <rcsc/common/logger.h>
#include <rcsc/common/server_param.h>
#include <rcsc/geom/ray_2d.h>

using namespace std;
using namespace rcsc;


static bool debug = false;
Bhv_Unmark::UnmarkPosition Bhv_Unmark::last_unmark_position = UnmarkPosition();

bool Bhv_Unmark::execute(PlayerAgent *agent) {
    const WorldModel &wm = agent->world();
    if (!can_unmarking(wm))
        return false;

    if (last_unmark_position.target.isValid()
        && last_unmark_position.last_run_cycle == wm.time().cycle() - 1
        && last_unmark_position.end_valid_cycle > wm.time().cycle()
        && last_unmark_position.target.x < wm.offsideLineX()) {
        dlog.addText(Logger::POSITIONING, "run last unmarking to (%.1f, %.1f)",
                     last_unmark_position.target.x, last_unmark_position.target.y);
        last_unmark_position.last_run_cycle = wm.time().cycle();
        if (run(agent, last_unmark_position)) {
            agent->debugClient().addMessage("Unmarking to (%.1f, %.1f)", last_unmark_position.target.x,
                                            last_unmark_position.target.y);
            return true;
        }
    }

    int passer = passer_finder(agent);
    dlog.addText(Logger::POSITIONING, "Should unmarking for %d", passer);
    if (passer == 0)
        return false;
    vector<Bhv_Unmark::UnmarkPosition> unmark_positions;
    simulate_dash(agent, passer, unmark_positions);

    if (unmark_positions.empty())
        return false;

    double max_eval = -1000;
    int best = -1; //-1=not 0=last other=other
    for (size_t i = 0; i < unmark_positions.size(); i++) {
        double ev = unmark_positions[i].eval;
        if (ev > max_eval) {
            best = i;
            max_eval = ev;
        }
    }
    if (best == -1)
        return false;

    last_unmark_position = unmark_positions[best];
    last_unmark_position.last_run_cycle = wm.time().cycle();
    last_unmark_position.end_valid_cycle = wm.time().cycle() + 5;
    if (run(agent, unmark_positions[best])) {
        agent->debugClient().addMessage("Unmarking to (%.1f, %.1f)", unmark_positions[best].target.x,
                                        unmark_positions[best].target.y);
        return true;
    }
    return false;
}

bool Bhv_Unmark::can_unmarking(const WorldModel &wm) {
    int mate_min = wm.interceptTable()->teammateReachCycle();
    int opp_min = wm.interceptTable()->opponentReachCycle();
    int unum = wm.self().unum();
    double stamina = wm.self().stamina();
    double dist2target = Strategy::instance().getPosition(unum).dist(wm.self().pos());
    int min_stamina_limit = 3500;
    if (wm.self().unum() >= 9) {
        if (wm.ball().pos().x > 30)
            min_stamina_limit = 2700;
        else if (wm.ball().pos().x > 10)
            min_stamina_limit = 3500;
        else if (wm.ball().pos().x > -30)
            min_stamina_limit = 5000;
        else if (wm.ball().pos().x > -55)
            min_stamina_limit = 6000;
    } else if (wm.self().unum() >= 6) {
        if (wm.ball().pos().x > 30)
            min_stamina_limit = 3000;
        else if (wm.ball().pos().x > 10)
            min_stamina_limit = 4000;
        else if (wm.ball().pos().x > -30)
            min_stamina_limit = 5000;
        else if (wm.ball().pos().x > -55)
            min_stamina_limit = 6000;
    } else {
        if (wm.ball().pos().x > 30)
            min_stamina_limit = 6000;
        else if (wm.ball().pos().x > 10)
            min_stamina_limit = 4000;
        else if (wm.ball().pos().x > -30)
            min_stamina_limit = 3500;
        else if (wm.ball().pos().x > -55)
            min_stamina_limit = 2500;
    }

    if (opp_min < mate_min || stamina < min_stamina_limit || dist2target > 10) {
        dlog.addText(Logger::POSITIONING,
                     "can not for opp cycle or stamina or dist");
        return false;
    }

    if (opp_min == mate_min)
        if (unum < 6) {
            dlog.addText(Logger::POSITIONING,
                         "can not for opp cycle or stamina or dist def");
            return false;
        }

    if (wm.self().isFrozen()) {
        dlog.addText(Logger::POSITIONING, "can not for frozen");
        return false;
    }
    if (wm.ball().inertiaPoint(mate_min).dist(wm.self().pos()) > 35)
        return false;
    return true;
}

int Bhv_Unmark::passer_finder(rcsc::PlayerAgent *agent) {
    const WorldModel &wm = agent->world();
    auto tm = wm.interceptTable()->fastestTeammate();
    if (tm != nullptr && tm->unum() > 0)
        return tm->unum();
    return 0;
}

void Bhv_Unmark::simulate_dash(rcsc::PlayerAgent *agent, int tm,
                               vector<Bhv_Unmark::UnmarkPosition> &unmark_positions) {
    const WorldModel &wm = agent->world();
    const AbstractPlayerObject *passer = wm.ourPlayer(tm);
    int mate_min = wm.interceptTable()->teammateReachCycle();

    Vector2D ball_pos = wm.ball().inertiaPoint(mate_min);
    Vector2D self_pos = wm.self().inertiaFinalPoint();
    Vector2D home_pos = Strategy::instance().getPosition(wm.self().unum());
    Vector2D passer_pos = passer->pos();
    Vector2D self_vel = wm.self().vel();
    AngleDeg self_body = wm.self().body().degree();

    const PlayerType *self_type = &(wm.self().playerType());
    double self_max_speed = self_type->realSpeedMax();
    double self_speed = self_vel.r();
    double offside_lineX = wm.offsideLineX();

    vector<Vector2D> positions;
    if (self_pos.dist(home_pos) < 5){
        for (double dist = 2.0; dist <= 7.0; dist += 1.0){
            for (double angle = -180; angle < 180; angle += 20){
                Vector2D position = self_pos + Vector2D::polar2vector(dist, angle);
                positions.push_back(position);
            }
        }
    }else{
        for (double dist = 3.0; dist <= 8.0; dist += 1){
            double center_angle = (home_pos - self_pos).th().degree();
            for (double angle = -30; angle < 30; angle += 10){
                Vector2D position = self_pos + Vector2D::polar2vector(dist, angle + center_angle);
                positions.push_back(position);
            }
        }
    }
    int position_id = 0;
    for (auto target: positions){
        position_id += 1;
        dlog.addText(Logger::POSITIONING, "# %d ##### (%.1f,%.1f)", position_id, target.x, target.y);
        dlog.addCircle(Logger::POSITIONING, target, 0.1);
        char num[8];
        snprintf(num, 8, "%d", position_id);
        dlog.addMessage(Logger::POSITIONING, target + Vector2D(0, 0), num);
        if (target.x > offside_lineX) {
            dlog.addCircle(Logger::POSITIONING, target, 0.5, 255, 0, 0);
            dlog.addText(Logger::POSITIONING, "---- more than offside");
            continue;
        }

        double home_max_dist = 7;

        if (target.dist(home_pos) > home_max_dist) {
            dlog.addCircle(Logger::POSITIONING, target, 0.5, 255, 0, 0);
            dlog.addText(Logger::POSITIONING, "---- far to home pos");
            continue;
        }

        double min_tm_dist =
                ServerParam::i().theirPenaltyArea().contains(target) ?
                5 : 8;
        if (nearest_tm_dist_to(wm, target) < min_tm_dist) {
            dlog.addCircle(Logger::POSITIONING, target, 0.5, 255, 0, 0);
            dlog.addText(Logger::POSITIONING, "---- close to tm");
            continue;
        }
        if (target.absX() > 52 || target.absY() > 31.5) {
            dlog.addCircle(Logger::POSITIONING, target, 0.5, 255, 0, 0);
            dlog.addText(Logger::POSITIONING, "---- out of field");
            continue;
        }

        vector<UnmakingPass> passes;
        lead_pass_simulator(wm, passer_pos, target, 0, passes);

        if (!passes.empty()) {
            double pos_eval = 0;
            UnmarkPosition new_pos(position_id, ball_pos, target, pos_eval, passes);
            pos_eval = evaluate_position(wm, new_pos);
            new_pos.eval = pos_eval;
            dlog.addCircle(Logger::POSITIONING, target, 0.5, 0, 0, 255);
            dlog.addText(Logger::POSITIONING, "---- OK (%.1f, %.1f) passes: %d eval: %.1f", target.x,
                         target.y, passes.size(), pos_eval);
            unmark_positions.push_back(new_pos);
        } else {
            dlog.addText(Logger::POSITIONING, "---- NOK no pass");
            dlog.addCircle(Logger::POSITIONING, target, 0.5, 0, 0, 0);
        }
    }
}

double Bhv_Unmark::nearest_tm_dist_to(const WorldModel &wm, Vector2D point) {

    double dist = 1000;
    for (auto &tm: wm.teammatesFromSelf()) {
        if (tm != nullptr && tm->unum() > 0) {
            if (!tm->pos().isValid())
                continue;
            if (dist > tm->pos().dist(point))
                dist = tm->pos().dist(point);
        }
    }
    return dist;
}

void Bhv_Unmark::lead_pass_simulator(const WorldModel &wm, Vector2D passer_pos,
                                     Vector2D unmark_target, int n_step, vector<UnmakingPass> &passes) {

    int mate_min = wm.interceptTable()->teammateReachCycle();
    Vector2D pass_start = wm.ball().inertiaPoint(mate_min);
    Vector2D current_self_pos = wm.self().pos();

    AngleDeg move_angle_to_receive_pass = (unmark_target - pass_start).th() + 90;
    int dist_step = 2;

    double distance = unmark_target.dist(pass_start);

    double pass_speed = 1.5;
    if (distance >= 20.0)
        pass_speed = 3.0;
    else if (distance >= 8.0)
        pass_speed = 2.7;
    else if (distance >= 5.0)
        pass_speed = 2.2;

    for (int i = -2; i <= 2; i++) {
        Vector2D pass_target = unmark_target
                               + Vector2D::polar2vector(i * dist_step, move_angle_to_receive_pass);
        int pass_cycle = pass_travel_cycle(pass_start, pass_speed, pass_target);
        int min_opp_cut_cycle = opponents_cycle_intercept(wm, pass_start, pass_speed,
                                                          pass_target);


        if (pass_cycle < min_opp_cut_cycle) {
            dlog.addText(Logger::POSITIONING,
                         "------pass_start(%.1f,%.1f), pass_target(%.1f,%.1f), self_cycle(%d), opp_cycle(%d) OK",
                         pass_start.x, pass_start.y, pass_target.x, pass_target.y,
                         pass_cycle, min_opp_cut_cycle);
            double pass_eval = pass_target.x + std::max(0.0, 40.0 - pass_target.dist(Vector2D(50.0, 0)));

            UnmakingPass pass_obj = UnmakingPass(pass_target, pass_speed,
                                                 pass_eval, pass_cycle);

            passes.push_back(pass_obj);
        } else {
            dlog.addText(Logger::POSITIONING,
                         "------pass_start(%.1f,%.1f), pass_target(%.1f,%.1f), self_cycle(%d), opp_cycle(%d) NOT OK",
                         pass_start.x, pass_start.y, pass_target.x, pass_target.y,
                         pass_cycle, min_opp_cut_cycle);
        }
    }

}

int Bhv_Unmark::pass_travel_cycle(Vector2D pass_start, double pass_speed, Vector2D &pass_target) {

    const ServerParam &SP = ServerParam::i();
    double travel_dist = 0;
    for (int cycle = 1; cycle < 50; cycle += 1) {
        travel_dist += pass_speed;
        if (travel_dist > pass_target.dist(pass_start))
            return cycle;
    }
    return 1000;
}

int Bhv_Unmark::opponents_cycle_intercept(const WorldModel &wm,
                                          Vector2D pass_start, double pass_speed,
                                          Vector2D pass_target) {
    int min_cycle = 1000;
    for (auto &opp: wm.opponentsFromSelf()) {
        if (opp == nullptr)
            continue;
        int opp_cycle = opponent_cycle_intercept(opp, pass_start, pass_speed, pass_target);
        if (min_cycle > opp_cycle)
            min_cycle = opp_cycle;
    }
    return min_cycle;

}

int Bhv_Unmark::opponent_cycle_intercept(const AbstractPlayerObject *opp, Vector2D pass_start, double pass_speed,
                                         Vector2D pass_target) {

    const ServerParam &SP = ServerParam::i();

    AngleDeg pass_angle = (pass_target - pass_start).th();

    Vector2D pass_start_vel = Vector2D::polar2vector(pass_speed, pass_angle);
    Vector2D opp_pos = (*opp).pos();

    const PlayerType *opp_type = (*opp).playerTypePtr();

    for (int cycle = 1; cycle < 50; cycle++) {
        const Vector2D ball_pos = inertia_n_step_point(pass_start,
                                                       pass_start_vel, cycle, SP.ballDecay());

        double dash_dist = ball_pos.dist(opp_pos);
        dash_dist -= 0.5;

        int n_dash = opp_type->cyclesToReachDistance(dash_dist) - 1;
        int n_step = n_dash;
        if (n_step <= cycle) {
            return cycle;
        }
    }
    return 50;
}

double Bhv_Unmark::evaluate_position(const WorldModel &wm, const UnmarkPosition &unmark_position) {
    double sum_eval = 0;
    double best_pass_eval = 0;
    double opp_eval = 10;
    for (auto &i: unmark_position.pass_list) {
        if (best_pass_eval < i.pass_eval)
            best_pass_eval = i.pass_eval;

        sum_eval += i.pass_eval;
    }

    for (auto &opp: wm.theirPlayers()) {
        if (opp != nullptr && opp->unum() > 0) {
            double opp_dist = opp->pos().dist(unmark_position.target);
            if (opp_dist < opp_eval)
                opp_eval = opp_dist;

        }
    }

    bool have_turn =
            ((unmark_position.target - wm.self().pos()).th() - wm.self().body()).abs() >= 15;
    bool up_pos =
            wm.self().unum() >= 6
            && (unmark_position.target - wm.self().pos()).th().abs() < 60;
    sum_eval /= unmark_position.pass_list.size();
    sum_eval += (sum_eval * unmark_position.pass_list.size() / 10);
    sum_eval += best_pass_eval;
    sum_eval += opp_eval;
    (!have_turn) ? sum_eval += 10 : sum_eval += 0;
    (up_pos) ? sum_eval += 10 : sum_eval += 0;
    return sum_eval;
}

bool Bhv_Unmark::run(PlayerAgent *agent, const UnmarkPosition &unmark_position) {
    const WorldModel &wm = agent->world();
    Vector2D target = unmark_position.target;
    Vector2D ball_pos = unmark_position.ball_pos;
    Vector2D me = wm.self().pos();
    Vector2D homePos = Strategy::i().getPosition(wm.self().unum());
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();

    double thr = 0.5;
    if (agent->world().self().inertiaPoint(1).dist(unmark_position.target) < thr) {
        AngleDeg bestAngle = (ball_pos - unmark_position.target).th() + 80;
        if (abs(bestAngle.degree()) > 90)
            bestAngle = (ball_pos - unmark_position.target).th() - 80;
        Body_TurnToAngle(bestAngle).execute(agent);
        agent->setNeckAction(new Neck_TurnToBallOrScan(0));
        return true;
    }
    dlog.addCircle(Logger::POSITIONING, target, 0.5, 0, 0, 255, true);
    double dash_power = (
            ball_pos.x > 30 && wm.self().stamina() > 6000 && wm.self().unum() > 6 ?
            100 : Strategy::get_normal_dash_power(agent->world()));

    return Body_GoToPoint(unmark_position.target, thr, dash_power).execute(agent);
}

