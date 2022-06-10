#ifndef TIGER_H
#define TIGER_H

#include <array>
#include <despot/interface/pomdp.h>

namespace despot {

using vvs = std::vector<std::vector<std::string>>;

/* =============================================================================
 * TiagoState class
 * =============================================================================
 * Objects:
 *      glass_cup - glass
 *      table0 table1 - table
 *      wp0 wp1 wp2 - waypoint
 *
 * Predicates:
 *      in_hand(?g)
 *      empty_hand()
 *      on_table(?g, ?t)
 *      robot_at(?wp)
 *      near(?wp, ?t)
 *      guest_not_near()
 *
 * Propositions:
 *      in_hand(glass_cup)
 *      empty_hand()
 *      on_table(glass_cup, table0)
 *      on_table(glass_cup, table1)
 *      robot_at(wp0)
 *      robot_at(wp1)
 *      robot_at(wp2)
 *      near(wp0, table0)
 *      near(wp0, table1)
 *      near(wp1, table0)
 *      near(wp1, table1)
 *      near(wp2, table0)
 *      near(wp2, table1)
 *      guest_not_near()
 * Actions:
 *      goto(?wp, ?wp)
 *      pick(?g, ?t, ?wp)
 *      place(?g, ?t, ?wp)
 *
 *      goto(wp0, wp1)
 *      goto(wp0, wp2)
 *      goto(wp1, wp0)
 *      goto(wp1, wp2)
 *      goto(wp2, wp0)
 *      goto(wp2, wp1)
 *      pick(glass_cup, table0, wp0)
 *      pick(glass_cup, table1, wp1)
 *      place(glass_cup, table0, wp0)
 *      place(glass_cup, table1, wp1)
 */

// using RoadState = State;

// class TiagoState : public State {
//   public:
//     TiagoState() : State(0, 1.0) {}
//     TiagoState(int state) : State(state, 1.0) {}

//     std::string text() const;
//     TiagoState &operator=(int pos) {
//         state_id = pos;
//         return *this;
//     };
//     TiagoState &operator--() {
//         --state_id;
//         return *this;
//     };
//     TiagoState &operator++() {
//         ++state_id;
//         return *this;
//     };
//     bool operator==(int pos) const { return state_id == pos; };
//     bool operator<(int pos) const { return state_id < pos; };
//     bool operator<=(int pos) const { return state_id <= pos; };
//     bool operator>(int pos) const { return state_id > pos; };
//     bool operator>=(int pos) const { return state_id >= pos; };
// };

/* =============================================================================
 * Road class
 * =============================================================================*/

class TiagoModel : public DSPOMDP {
  private:
    mutable MemoryPool<State> memory_pool_;

  public:
    static const std::vector<std::vector<std::string>> actions_;
    static const std::vector<std::vector<std::string>> propositions_;
    static const std::vector<std::vector<std::string>> obs_;

    static constexpr double INF = 100.0;

    TiagoModel();
    TiagoModel(std::string params_file);

    bool Step(State &s, double random_num, ACT_TYPE action, double &reward,
              OBS_TYPE &obs) const;
    int NumStates() const;
    int NumActions() const;
    double ObsProb(OBS_TYPE obs, const State &s, ACT_TYPE a) const;

    State *CreateStartState(std::string type) const;
    Belief *InitialBelief(const State *start,
                          std::string type = "DEFAULT") const;

    inline double GetMaxReward() const { return 2 * INF; }

    inline ValuedAction GetBestAction() const {
        return ValuedAction(0, -1);
    }
    ScenarioLowerBound *
    CreateScenarioLowerBound(std::string name = "DEFAULT",
                             std::string particle_bound_name = "DEFAULT") const;

    void PrintState(const State &state, std::ostream &out = std::cout) const;
    void PrintBelief(const Belief &belief, std::ostream &out = std::cout) const;
    void PrintObs(const State &state, OBS_TYPE obs,
                  std::ostream &out = std::cout) const;
    void PrintAction(ACT_TYPE action, std::ostream &out = std::cout) const;

    State *Allocate(int state_id, double weight) const;
    State *Copy(const State *particle) const;
    void Free(State *particle) const;
    int NumActiveParticles() const;

    static int indexOf(const vvs &v, const std::vector<std::string> &p);

    double Reward(const State& state, ACT_TYPE action) const override;
};

} // namespace despot

#endif
