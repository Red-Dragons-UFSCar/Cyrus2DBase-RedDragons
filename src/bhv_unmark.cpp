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
Vector2D Bhv_Unmark::target_pos = Vector2D(0, 0);

bool Bhv_Unmark::execute(PlayerAgent * agent){
    const WorldModel & wm = agent->world();

    vector<Bhv_Unmark::UnmarkPosition> unmark_positions;

    if (!canIpos(agent)) {
        return false;
    }

    if(last_cycle > 0&&target_pos.x<wm.offsideLineX()){
        last_cycle--;
        Arm_PointToPoint(target_pos).execute(agent);

        return Body_GoToPoint(target_pos, 0.2, 100).execute(agent);

    }

    int passer = passer_finder(agent);

    if (passer == 0)
        return false;

    simulate_dash(agent, passer, unmark_positions);

    if (unmark_positions.size() == 0) {
        return false;
    }

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
        return true;
    }
    return false;
}

bool Bhv_Unmark::canIpos(PlayerAgent * agent){

    const WorldModel & wm = agent->world();

    int mate_min = wm.interceptTable()->teammateReachCycle();
    int opp_min = wm.interceptTable()->opponentReachCycle();
    int unum = wm.self().unum();
    int stamina = wm.self().stamina();
    double dist2target = Strategy::instance().getPosition(unum).dist(
            wm.self().pos());

    int max_stamina = 3000;

    if (wm.self().unum() > 8) {
        if (wm.ball().pos().x > 30)
            max_stamina = 2700;
        else if (wm.ball().pos().x > 10)
            max_stamina = 3000;
        else if (wm.ball().pos().x > -30)
            max_stamina = 3500;
        else if (wm.ball().pos().x > -55)
            max_stamina = 4000;
    } else if (wm.self().unum() < 7) {
        if (wm.ball().pos().x > 30)
            max_stamina = 5000;
        else if (wm.ball().pos().x > 10)
            max_stamina = 4000;
        else if (wm.ball().pos().x > -30)
            max_stamina = 3300;
        else if (wm.ball().pos().x > -55)
            max_stamina = 2500;
    } else {
        if (wm.ball().pos().x > 30)
            max_stamina = 3000;
        else if (wm.ball().pos().x > 10)
            max_stamina = 3500;
        else if (wm.ball().pos().x > -30)
            max_stamina = 3300;
        else if (wm.ball().pos().x > -55)
            max_stamina = 2000;
    }

    if (opp_min < mate_min || stamina < max_stamina || dist2target > 10) {
        if (debug)
            dlog.addText(Logger::POSITIONING,
                         "can not for opp cycle or stamina or dist");
        return false;
    }

    if (opp_min == mate_min)
        if (unum < 6) {
            if (debug)
                dlog.addText(Logger::POSITIONING,
                             "can not for opp cycle or stamina or dist def");
            return false;
        }

    if (wm.self().isFrozen()) {
        if (debug)
            dlog.addText(Logger::POSITIONING, "can not for frozen");
        return false;
    }

    return true;
}

int Bhv_Unmark::passer_finder(rcsc::PlayerAgent *agent){
    const WorldModel & wm = agent->world();
    auto tm = wm.interceptTable()->fastestTeammate();
    if (tm != nullptr && tm->unum() > 0)
        return tm->unum();
    return 0;
}

void Bhv_Unmark::simulate_dash(rcsc::PlayerAgent * agent, int tm,
                                 vector<Bhv_Unmark::UnmarkPosition> & unmark_positions){
    const WorldModel & wm = agent->world();
    const AbstractPlayerObject * passer = wm.ourPlayer(tm);
    int mate_min = wm.interceptTable()->teammateReachCycle();

    Vector2D ball_pos = wm.ball().inertiaPoint(mate_min);
    Vector2D self_pos = wm.self().inertiaFinalPoint();
    Vector2D home_pos = Strategy::instance().getPosition(wm.self().unum());
    Vector2D passer_pos = passer->pos();
    Vector2D self_vel = wm.self().vel();
    AngleDeg self_body = wm.self().body().degree();

    const PlayerType * self_type = &(wm.self().playerType());
    double self_max_speed = self_type->realSpeedMax();
    double self_speed = self_vel.r();
    double offside_lineX = wm.offsideLineX();

    if (ball_pos.dist(self_pos) > 35)
        return;

    int max_dash = 10;
    int angle_divs = 10;
    double angle_step = 360.0 / angle_divs;

    for (int i = 0; i < angle_divs; i++) {

        AngleDeg angle = i * angle_step + self_body;

        int n_turn = FieldAnalyzer::predict_player_turn_cycle(self_type, self_body, self_speed,
                                               5, angle, 0, false);
        int n_dash = 0;
        Vector2D new_self_pos = self_pos;

        double speed = (n_turn == 0 ? self_speed : 0);
        double accel = ServerParam::i().maxDashPower()
                       * self_type->dashPowerRate() * self_type->effortMax();

        for (int n_step = n_turn; n_step < max_dash; n_step++) {

            n_dash++;
            if (speed + accel > self_max_speed) {
                accel = self_max_speed - speed;
            }

            speed += accel;
            new_self_pos += Vector2D::polar2vector(speed, angle);

            speed *= self_type->playerDecay();

            if (new_self_pos.x > offside_lineX) {
                continue;
            }

            double home_max_dist = 10;
            if (wm.self().unum() == 11
                || (wm.self().unum() >= 6 && wm.ball().pos().x > 0))
                home_max_dist = 7;

            if (new_self_pos.dist(home_pos) > home_max_dist)
                continue;

            double min_tm_dist =
                    ServerParam::i().theirPenaltyArea().contains(new_self_pos) ?
                    5 : 8;
            if (nearest_tm_dist_to(wm, new_self_pos) < min_tm_dist)
                continue;
            if (new_self_pos.absX() > 52 || new_self_pos.absY() > 31.5)
                continue;
            int opp_danger_cycle = wm.interceptTable()->opponentReachCycle();
            if ((opp_danger_cycle - mate_min) < n_step - 2)
                continue;
            vector<passpos> passes;
            lead_pass_simulator(wm, passer_pos, new_self_pos, n_step, passes);

            if (!passes.empty()) {
                double pos_eval = 0;
                UnmarkPosition new_pos(ball_pos, tm, new_self_pos, n_step,
                                       leadPos, true, true, pos_eval, passes);
                pos_eval = evaluate_position(wm, new_pos);
                new_pos.eval = pos_eval;
                unmark_positions.push_back(new_pos);
            }
        }
    }
}

double Bhv_Unmark::nearest_tm_dist_to(const WorldModel & wm, Vector2D point){

    double dist = 1000;
    for (auto & tm: wm.ourPlayers()){
        if (tm != nullptr && tm->unum() > 0){
            if (!tm->pos().isValid())
                continue;
            if (dist > tm->pos().dist(point))
                dist = tm->pos().dist(point);
        }
    }
    return dist;
}

void Bhv_Unmark::lead_pass_simulator(const WorldModel & wm, Vector2D passer_pos,
                                     Vector2D new_self_pos, int n_step, vector<passpos> & passes){

    int mate_min = wm.interceptTable()->teammateReachCycle();
    Vector2D pass_start = wm.ball().inertiaPoint(mate_min);
    Vector2D current_self_pos = wm.self().pos();
    
    AngleDeg self_dist_angle = (new_self_pos - pass_start).th() + 90;
    int dist_step = 2;
    
    double distance = new_self_pos.dist(pass_start);

    double pass_speed = 1.5;
    if (distance >= 20.0)
        pass_speed = 2.7;
    else if (distance >= 8.0)
        pass_speed = 2.5;
    else if (distance >= 5.0)
        pass_speed = 2.0;

    for (int i = -2; i <= 2; i++) {
        Vector2D pass_target = new_self_pos
                               + Vector2D::polar2vector(i * dist_step, self_dist_angle);
        double self_body = (new_self_pos - current_self_pos).th().degree();
        double self_speed = 0.3;
        int pass_cycle = self_cycle_intercept(wm, pass_start, pass_speed,
                                              pass_target, self_body, self_speed);
        int min_opp_cut_cycle = opponents_cycle_intercept(wm, pass_start, pass_speed,
                                                    pass_cycle, pass_target);
        if (debug)
            dlog.addText(Logger::POSITIONING,
                         "      pass_start(%.1f,%.1f),pass_target(%.1f,%.1f),self_cycle(%d),opp_cycle(%d)",
                         pass_start.x, pass_start.y, pass_target.x, pass_target.y,
                         pass_cycle, min_opp_cut_cycle);

        if (pass_cycle < min_opp_cut_cycle) {

            double pass_eval = pass_target.x + std::max(0.0, 40.0 - pass_target.dist(Vector2D(50.0, 0)));

            if (debug)
                dlog.addText(Logger::POSITIONING, "         eval(%.1f)",
                             pass_eval);

            passpos directpass = passpos(pass_target, pass_speed, true,
                                         pass_eval, pass_cycle);

            passes.push_back(directpass);
        }
    }

}

int Bhv_Unmark::self_cycle_intercept(const WorldModel & wm,
                                       Vector2D pass_start, double pass_speed, Vector2D & pass_target,
                                       double self_body, double speed){

    const ServerParam & SP = ServerParam::i();

    AngleDeg pass_angle = (pass_target - pass_start).th();

    Vector2D pass_start_vel = Vector2D::polar2vector(pass_speed, pass_angle);

    const PlayerType * self_type = &(wm.self().playerType());

    for (int cycle = 1; cycle < 50; cycle++) {
        const Vector2D ball_pos = inertia_n_step_point(pass_start,
                                                       pass_start_vel, cycle, SP.ballDecay());

        double dash_dist = ball_pos.dist(pass_target);
        dash_dist += 0.5;

        int n_turn = abs(pass_angle.degree() - self_body) > 15 ? 1 : 0;
        int n_dash = self_type->cyclesToReachDistance(dash_dist);

        int n_step = n_dash + n_turn;

        if (speed > 0.25 && n_turn == 0)
            n_step--;

        if (n_step <= cycle) {
            pass_target = ball_pos;
            return cycle;
        }
    }
    return 51;
}

int Bhv_Unmark::opponents_cycle_intercept(const WorldModel & wm,
                                       Vector2D pass_start, double pass_speed, int pass_cycle,
                                       Vector2D pass_target){

    int min_cycle = 1000;

    for (auto & opp: wm.opponentsFromSelf()){
        if (opp == nullptr)
            continue;
        int opp_cycle = opponent_cycle_intercept(wm, opp, pass_start, pass_speed,
                                                    pass_cycle, pass_target);

        if (min_cycle > opp_cycle)
            min_cycle = opp_cycle;

    }
    return min_cycle;

}

int Bhv_Unmark::opponent_cycle_intercept(const WorldModel & wm,
                                            const AbstractPlayerObject * opp, Vector2D pass_start, double pass_speed,
                                            int pass_cycle, Vector2D pass_target){

    const ServerParam & SP = ServerParam::i();

    AngleDeg pass_angle = (pass_target - pass_start).th();

    Vector2D pass_start_vel = Vector2D::polar2vector(pass_speed, pass_angle);
    Vector2D opp_pos = (*opp).pos();

    const PlayerType * opp_type = (*opp).playerTypePtr();

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

double Bhv_Unmark::evaluate_position(const WorldModel & wm, UnmarkPosition pos){
    double sumEval = 0;
    double best_pass_eval = 0;
    double opp_eval = 10;
    for (int i = 0; i <= pos.passposvector.size(); i++) {

        if (best_pass_eval < pos.passposvector[i].eval)
            best_pass_eval = pos.passposvector[i].eval;

        sumEval += pos.passposvector[i].eval;
    }

    for (auto & opp: wm.theirPlayers()){
        if (opp != nullptr && opp->unum() > 0){
            double opp_dist = opp->pos().dist(pos.target);
            if (opp_dist < opp_eval)
                opp_eval = opp_dist;

        }
    }

    bool have_turn =
            ((pos.target - wm.self().pos()).th() - wm.self().body()).abs()
            < 15 ? false : true;
    bool up_pos =
            wm.self().unum() >= 6
            && (pos.target - wm.self().pos()).th().abs() < 60 ?
            true : false;
    sumEval /= pos.passposvector.size();
    sumEval += (sumEval * pos.passposvector.size() / 10);
    sumEval += best_pass_eval;
    sumEval += opp_eval;
    (!have_turn) ? sumEval += 10 : sumEval += 0;
    (up_pos) ? sumEval += 10 : sumEval += 0;
    return sumEval;
}

bool Bhv_Unmark::run(PlayerAgent * agent, UnmarkPosition Pos){
    const WorldModel & wm = agent->world();
    Vector2D ball = wm.ball().pos();
    Vector2D me = wm.self().pos();
    Vector2D homePos = Strategy::i().getPosition(wm.self().unum());
    const int self_min = wm.interceptTable()->selfReachCycle();
    const int mate_min = wm.interceptTable()->teammateReachCycle();
    const int opp_min = wm.interceptTable()->opponentReachCycle();
    const int our_min = std::min(self_min, mate_min);

    if (wm.self().unum() > 8 && ball.x < 40 && ball.x > -20
        && (wm.self().unum() < 11 || ball.x < 48) && me.dist(homePos) < 10
        && wm.self().pos().x < wm.offsideLineX() - 0.3
        && // wm.self().pos().x > wm.offsideLineX() - 3.0 &&
        homePos.x > wm.offsideLineX() - 10.0 && wm.self().body().abs() < 15.0
        && mate_min < opp_min && self_min > mate_min
        && wm.self().stamina() > 4000) {
        agent->doDash(100, 0.0);
        target_pos = me+Vector2D(5,0);
        last_cycle = 3;
        agent->setNeckAction(new Neck_TurnToBallOrScan(0));
//		Arm_PointToPoint(me+Vector2D(10,0)).execute(agent);
        return true;
    }
    if (wm.self().unum() > 8 && ball.x < 40 && ball.x > -20
        && (wm.self().unum() < 11 || ball.x < 48) && me.dist(homePos) < 10
        && wm.self().pos().x < wm.offsideLineX() - 0.3
        && // wm.self().pos().x > wm.offsideLineX() - 3.0 &&
        homePos.x > wm.offsideLineX() - 10.0 && mate_min < opp_min
        && self_min > mate_min && wm.self().stamina() > 4000) {
        Body_TurnToAngle(0).execute(agent);
        last_cycle = 0;
        agent->setNeckAction(new Neck_TurnToBallOrScan(0));
        return true;
    }

    double thr = 0.5;
    if (agent->world().self().inertiaPoint(1).dist(Pos.target) < thr) {
//		//Arm_PointToPoint(agent->world().self().pos()+Vector2D::polar2vector(100,(Pos.ballpos-Pos.target).th()+80)).execute(agent);
        AngleDeg bestAngle = (Pos.ballpos - Pos.target).th() + 80;
        if (abs(bestAngle.degree()) > 90)
            bestAngle = (Pos.ballpos - Pos.target).th() - 80;
        last_cycle = 0;
        Body_TurnToAngle(bestAngle).execute(agent);
        return true;
    }
    Arm_PointToPoint(Pos.target).execute(agent);
    double enerji = (
            Pos.target.x > 30 ?
            100 : Strategy::get_normal_dash_power(agent->world()));
    if (Pos.target.x > 30)
        agent->addSayMessage(
                new SelfMessage(agent->world().self().pos(),
                                agent->world().self().body(),
                                agent->world().self().stamina()));

    if (Pos.target.x > 30)
        thr = 0.2;
    last_cycle = 3;
    target_pos = Pos.target;
    return Body_GoToPoint(Pos.target, thr, enerji).execute(agent);
}
