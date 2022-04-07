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
int Bhv_Unmark::last_cycle = 0;
Vector2D Bhv_Unmark::last_target_pos = Vector2D(0, 0);

bool Bhv_Unmark::execute(PlayerAgent *agent) {
    const WorldModel &wm = agent->world();
    if (!can_unmarking(wm))
        return false;

    if (last_cycle > 0 && last_target_pos.x < wm.offsideLineX()) {
        last_cycle--;
        dlog.addText(Logger::POSITIONING, "run last unmarking to (%.1f, %.1f)", last_target_pos.x, last_target_pos.y);
        return Body_GoToPoint(last_target_pos, 0.2, 100).execute(agent);
    }

    int passer = passer_finder(agent);
    dlog.addText(Logger::POSITIONING, "Should unmarking for %d", passer);
    if (passer == 0)
        return false;
    vector<Bhv_Unmark::UnmarkPosition> unmark_positions;
    simulate_dash(agent, passer, unmark_positions);

    if (unmark_positions.empty())
        return false;

    double max_eval = 0;
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
            min_stamina_limit = 6000;
        else if (wm.ball().pos().x > -55)
            min_stamina_limit = 7000;
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

    int max_dash = 10;
    int angle_divs = 10;
    double angle_step = 360.0 / angle_divs;

    int position_id = 0;
    for (int i = 0; i < angle_divs; i++) {

        AngleDeg angle = i * angle_step + self_body;

        int n_turn = FieldAnalyzer::predict_player_turn_cycle(self_type, self_body, self_speed,
                                                              5, angle, 0, false);
        int n_dash = 0;
        Vector2D target = self_pos;

        double speed = (n_turn == 0 ? self_speed : 0);
        double accel = ServerParam::i().maxDashPower()
                       * self_type->dashPowerRate() * self_type->effortMax();

        for (int n_step = n_turn; n_step < max_dash; n_step++) {

            n_dash++;
            if (speed + accel > self_max_speed) {
                accel = self_max_speed - speed;
            }

            speed += accel;
            target += Vector2D::polar2vector(speed, angle);

            speed *= self_type->playerDecay();

            if (target.dist(self_pos) < 1)
                continue;

            if (target.x > offside_lineX) {
                dlog.addCircle(Logger::POSITIONING, target, 0.5, 255, 0, 0);
                continue;
            }

            double home_max_dist = 7;

            if (target.dist(home_pos) > home_max_dist) {
                dlog.addCircle(Logger::POSITIONING, target, 0.5, 255, 0, 0);
                continue;
            }

            double min_tm_dist =
                    ServerParam::i().theirPenaltyArea().contains(target) ?
                    5 : 8;
            if (nearest_tm_dist_to(wm, target) < min_tm_dist) {
                dlog.addCircle(Logger::POSITIONING, target, 0.5, 255, 0, 0);
                continue;
            }
            if (target.absX() > 52 || target.absY() > 31.5) {
                dlog.addCircle(Logger::POSITIONING, target, 0.5, 255, 0, 0);
                continue;
            }

            vector<UnmakingPass> passes;
            lead_pass_simulator(wm, passer_pos, target, n_step, passes);

            if (!passes.empty()) {
                double pos_eval = 0;
                UnmarkPosition new_pos(ball_pos, target, pos_eval, passes);
                pos_eval = evaluate_position(wm, new_pos);
                new_pos.eval = pos_eval;
                char num[8];
                snprintf(num, 8, "%d", position_id);
                dlog.addMessage(Logger::POSITIONING, target + Vector2D(0, 0), num);
                dlog.addCircle(Logger::POSITIONING, target, 0.5, 0, 0, 255);
                dlog.addText(Logger::POSITIONING, "##%d (%.1f, %.1f) passes: %d eval: %.1f", position_id, target.x,
                             target.y, passes.size(), pos_eval);
                position_id++;
                unmark_positions.push_back(new_pos);
            } else {
                dlog.addCircle(Logger::POSITIONING, target, 0.5, 0, 0, 0);
            }
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
    Vector2D ball = wm.ball().pos();
    Vector2D me = wm.self().pos();
    Vector2D homePos = Strategy::i().getPosition(wm.self().unum());
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();

    if (wm.self().unum() > 8 && ball.x < 40 && ball.x > -20
        && (wm.self().unum() < 11 || ball.x < 48) && me.dist(homePos) < 10
        && wm.self().pos().x < wm.offsideLineX() - 0.3
        && homePos.x > wm.offsideLineX() - 10.0 && wm.self().body().abs() < 15.0
        && mate_min < opp_min && self_min > mate_min
        && wm.self().stamina() > 4000) {
        agent->doDash(100, 0.0);
        last_target_pos = me + Vector2D(5, 0);
        last_cycle = 3;
        agent->setNeckAction(new Neck_TurnToBallOrScan(0));
        return true;
    }
    if (wm.self().unum() > 8 && ball.x < 40 && ball.x > -20
        && (wm.self().unum() < 11 || ball.x < 48) && me.dist(homePos) < 10
        && wm.self().pos().x < wm.offsideLineX() - 0.3
        && homePos.x > wm.offsideLineX() - 10.0 && mate_min < opp_min
        && self_min > mate_min && wm.self().stamina() > 4000) {
        Body_TurnToAngle(0).execute(agent);
        last_cycle = 0;
        agent->setNeckAction(new Neck_TurnToBallOrScan(0));
        return true;
    }

    double thr = 0.5;
    if (agent->world().self().inertiaPoint(1).dist(unmark_position.target) < thr) {
        AngleDeg bestAngle = (unmark_position.ball_pos - unmark_position.target).th() + 80;
        if (abs(bestAngle.degree()) > 90)
            bestAngle = (unmark_position.ball_pos - unmark_position.target).th() - 80;
        last_cycle = 0;
        Body_TurnToAngle(bestAngle).execute(agent);
        return true;
    }
    double dash_power = (
            unmark_position.target.x > 30 ?
            100 : Strategy::get_normal_dash_power(agent->world()));
    if (unmark_position.target.x > 30)
        agent->addSayMessage(
                new SelfMessage(agent->world().self().pos(),
                                agent->world().self().body(),
                                agent->world().self().stamina()));

    if (unmark_position.target.x > 30)
        thr = 0.2;
    last_cycle = 3;
    last_target_pos = unmark_position.target;
    return Body_GoToPoint(unmark_position.target, thr, dash_power).execute(agent);
}
