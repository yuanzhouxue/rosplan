
#include <bitset>
#include <despot/core/builtin_lower_bounds.h>
#include <despot/core/builtin_policy.h>
#include <despot/core/builtin_upper_bounds.h>
#include <despot/core/particle_belief.h>

#include "tiago_pick_nav.h"

using namespace std;

namespace despot {

const std::vector<std::vector<std::string>> TiagoModel::actions_ = {
    {"goto", "wp0", "wp1"},     {"goto", "wp0", "wp2"},
    {"goto", "wp1", "wp0"},     {"goto", "wp1", "wp2"},
    {"goto", "wp2", "wp0"},     {"goto", "wp2", "wp1"},
    {"pick", "table0", "wp0"},  {"pick", "table1", "wp1"},
    {"place", "table0", "wp0"}, {"place", "table1", "wp1"}};

const std::vector<std::vector<std::string>> TiagoModel::propositions_ = {
    {"in_hand"},
    {"empty_hand"},
    {"on_table", "table0"},
    {"on_table", "table1"},
    {"robot_at", "wp0"},
    {"robot_at", "wp1"},
    {"robot_at", "wp2"},
    {"near", "wp0", "table0"},
    {"near", "wp0", "table1"},
    {"near", "wp1", "table0"},
    {"near", "wp1", "table1"},
    {"near", "wp2", "table0"},
    {"near", "wp2", "table1"},
    {"guest_not_near"}};

const vector<vector<string>> TiagoModel::obs_ = {
    {"robot_at", "wp0"},   {"robot_at", "wp1"}, {"robot_at", "wp2"},
    {"in_hand"},           {"empty_hand"},      {"on_table", "table0"},
    {"on_table", "table1"}};

TiagoModel::TiagoModel() {}

bool TiagoModel::Step(State &s, double random_num, ACT_TYPE action,
                      double &reward, OBS_TYPE &obs) const {
    bool terminal = false;
    reward = 1;

    const auto &actionName = actions_[action][0];
    if (actionName == "goto") {
        reward = -1;
        // 移动动作
        const auto &paraFrom = actions_[action][1];
        const auto &paraTo = actions_[action][2];
        auto cond1 = 1 << indexOf(propositions_, {"robot_at", paraFrom});
        if (cond1 != (s.state_id & cond1)) {
            reward = -INF;
            obs = 0;
            // terminal = true;
        } else {
            obs = 1 << indexOf(obs_, {"robot_at", paraTo});
            s.state_id |= 1 << indexOf(propositions_, {"robot_at", paraTo});
            s.state_id &=
                ~(1 << indexOf(propositions_, {"robot_at", paraFrom}));
        }
    } else if (actionName == "pick") {
        const auto &paraTable = actions_[action][1];
        const auto &paraWaypoint = actions_[action][2];
        auto cond1 = 1 << indexOf(propositions_, {"robot_at", paraWaypoint});
        auto cond2 =
            1 << indexOf(propositions_, {"near", paraWaypoint, paraTable});
        auto cond3 = 1 << indexOf(propositions_, {"on_table", paraTable});
        auto cond4 = 1 << indexOf(propositions_, {"empty_hand"});
        auto cond = cond1 | cond2 | cond3 | cond4;
        if ((s.state_id & cond) != cond) {
            reward = -INF;
            obs = 0;
            // terminal = true;
        } else {
            // effects: -in_hand, empty_hand, on_table(table)
            auto eff1 = indexOf(propositions_, {"in_hand"});
            auto eff2 = indexOf(propositions_, {"empty_hand"});
            auto eff3 = indexOf(propositions_, {"on_table", paraTable});
            obs = 1 << indexOf(obs_, {"in_hand"});

            s.state_id |= 1 << eff1;
            s.state_id &= ~((1 << eff2) | (1 << eff3));
        }
    } else {
        // place
        const auto &paraTable = actions_[action][1];
        const auto &paraWaypoint = actions_[action][2];
        auto cond1 = 1 << indexOf(propositions_, {"robot_at", paraWaypoint});
        auto cond2 =
            1 << indexOf(propositions_, {"near", paraWaypoint, paraTable});
        auto cond3 = 1 << indexOf(propositions_, {"in_hand"});
        auto cond4 = 1 << indexOf(propositions_, {"guest_not_near"});
        auto cond = cond1 | cond2 | cond3 | cond4;
        if ((s.state_id & cond) != cond) {
            reward = -INF;
            obs = 0;
            // terminal = true;
        } else {
            // effects: -in_hand, empty_hand, on_table(table)
            auto eff1 = indexOf(propositions_, {"in_hand"});
            auto eff2 = indexOf(propositions_, {"empty_hand"});
            auto eff3 = indexOf(propositions_, {"on_table", paraTable});
            obs = (1 << indexOf(obs_, {"empty_hand"})) |
                  (1 << indexOf(obs_, {"on_table", paraTable}));
            s.state_id |= (1 << eff2) | (1 << eff3);
            s.state_id &= ~(1 << eff1);
        }
    }

    auto goal1 = 1 << indexOf(propositions_, {"on_table", "table1"});
    auto goal2 = 1 << indexOf(propositions_, {"robot_at", "wp2"});
    auto goal = goal1 | goal2;
    // auto goal = 1 << indexOf(propositions_, {"in_hand"});
    if ((s.state_id & goal) == goal) {
        reward = INF;
        terminal = true;
    }
    return terminal;
}

int TiagoModel::NumStates() const {
    int numPropositions = std::end(propositions_) - std::begin(propositions_);
    return 1 << numPropositions;
}

int TiagoModel::NumActions() const {
    return std::end(actions_) - std::begin(actions_);
}

double TiagoModel::ObsProb(OBS_TYPE obs, const State &s, ACT_TYPE a) const {
    if (obs >= (1 << obs_.size()))
        return 0.0;
    const auto &actionName = actions_[a][0];
    if (actionName == "goto") {
        const auto &paraFrom = actions_[a][1];
        const auto &paraTo = actions_[a][2];
        auto cond = 1 << indexOf(propositions_, {"robot_at", paraFrom});
        if (cond != (s.state_id & cond))
            return obs == 0;
        else {
            int obs1 = 1 << indexOf(obs_, {"robot_at", paraTo});
            int obs2 = 1 << indexOf(obs_, {"robot_at", paraFrom});
            if (obs == obs1)
                return 0.9;
            else if (obs == obs2)
                return 0.1;
            else
                return 0.0;
        }
    } else if (actionName == "pick") {
        const auto &paraTable = actions_[a][1];
        const auto &paraWaypoint = actions_[a][2];
        auto cond1 = 1 << indexOf(propositions_, {"robot_at", paraWaypoint});
        auto cond2 =
            1 << indexOf(propositions_, {"near", paraWaypoint, paraTable});
        auto cond3 = 1 << indexOf(propositions_, {"on_table", paraTable});
        auto cond4 = 1 << indexOf(propositions_, {"empty_hand"});
        auto cond = cond1 | cond2 | cond3 | cond4;
        if ((s.state_id & cond) != cond)
            return obs == 0;
        else {
            auto obs1 = 1 << indexOf(obs_, {"in_hand"});
            auto obs2 = (1 << indexOf(obs_, {"empty_hand"})) | (1 << indexOf(obs_, {"on_table", paraTable}));

            if (obs == obs1)
                return 0.8;
            else if (obs == obs2)
                return 0.2;
            else 0.0;
        }
    } else {
        const auto &paraTable = actions_[a][1];
        const auto &paraWaypoint = actions_[a][2];
        auto cond1 = 1 << indexOf(propositions_, {"robot_at", paraWaypoint});
        auto cond2 =
            1 << indexOf(propositions_, {"near", paraWaypoint, paraTable});
        auto cond3 = 1 << indexOf(propositions_, {"in_hand"});
        auto cond4 = 1 << indexOf(propositions_, {"guest_not_near"});
        auto cond = cond1 | cond2 | cond3 | cond4;
        if ((s.state_id & cond) != cond)
            return obs == 0;
        else {
            int obs1 = (1 << indexOf(obs_, {"empty_hand"})) | (1 << indexOf(obs_, {"on_table", paraTable}));
            int obs2 = 1 << indexOf(obs_, {"in_hand"});
            if (obs == obs1)
                return 0.8;
            else if (obs == obs2)
                return 0.2;
            else return 0.0;
        }
    }
}

State *TiagoModel::CreateStartState(string type) const {
    int initial_state = 0;
    initial_state |= 1 << indexOf(propositions_, {"robot_at", "wp2"});
    initial_state |= 1 << indexOf(propositions_, {"empty_hand"});
    initial_state |= 1 << indexOf(propositions_, {"guest_not_near"});
    initial_state |= 1 << indexOf(propositions_, {"near", "wp0", "table0"});
    initial_state |= 1 << indexOf(propositions_, {"near", "wp1", "table1"});
    initial_state |= 1 << indexOf(propositions_, {"on_table", "table0"});
    return new State(initial_state, 1.0);
}

Belief *TiagoModel::InitialBelief(const State *start, string type) const {
    // 初始状态
    vector<State *> particles;
    int initial_state = 0;
    initial_state |= 1 << indexOf(propositions_, {"robot_at", "wp2"});
    initial_state |= 1 << indexOf(propositions_, {"empty_hand"});
    initial_state |= 1 << indexOf(propositions_, {"guest_not_near"});
    initial_state |= 1 << indexOf(propositions_, {"near", "wp0", "table0"});
    initial_state |= 1 << indexOf(propositions_, {"near", "wp1", "table1"});
    initial_state |= 1 << indexOf(propositions_, {"on_table", "table0"});

    auto pos = static_cast<State *>(Allocate(initial_state, 1.0));
    particles.push_back(pos);
    return new ParticleBelief(particles, this);
}

ScenarioLowerBound *
TiagoModel::CreateScenarioLowerBound(string name,
                                     string particle_bound_name) const {
    ScenarioLowerBound *bound = NULL;
    if (name == "TRIVIAL" || name == "DEFAULT") {
        bound = new TrivialParticleLowerBound(this);
    } else if (name == "RANDOM") {
        bound = new RandomPolicy(this,
                                 CreateParticleLowerBound(particle_bound_name));
    } else if (name == "OPTIMAL") {
        // bound = new OptimalTigerPolicy(
        //     this, CreateParticleLowerBound(particle_bound_name));
        cerr << "Unsupported scenario lower bound: " << name << endl;
        exit(1);
    } else {
        cerr << "Unsupported scenario lower bound: " << name << endl;
        exit(1);
    }
    return bound;
}

void TiagoModel::PrintState(const State &state, ostream &out) const {
    for (int i = 0; i < propositions_.size(); ++i) {
        if (state.state_id & (1 << i)) {
            out << "" << propositions_[i] << " ";
        }
    }
    out << endl;
}

void TiagoModel::PrintBelief(const Belief &belief, ostream &out) const {
    const ParticleBelief &pb = static_cast<const ParticleBelief &>(belief);
    for (const auto &p : pb.particles()) {
        out << "Weight: " << p->weight << " ";
        PrintState(*p, out);
    }
}

void TiagoModel::PrintObs(const State &state, OBS_TYPE obs,
                          ostream &out) const {

    // out << "State: ";
    // PrintState(state, out);
    for (int i = 0; i < obs_.size(); ++i) {
        if (obs & (1 << i)) {
            out << obs_[i] << " ";
        }
    }
    out << endl;
}

void TiagoModel::PrintAction(ACT_TYPE action, ostream &out) const {
    out << actions_[action] << endl;
}

State *TiagoModel::Allocate(int state_id, double weight) const {
    State *particle = memory_pool_.Allocate();
    particle->state_id = state_id;
    particle->weight = weight;
    return particle;
}

State *TiagoModel::Copy(const State *particle) const {
    State *new_particle = memory_pool_.Allocate();
    *new_particle = *static_cast<const State *>(particle);
    new_particle->SetAllocated();
    return new_particle;
}

void TiagoModel::Free(State *particle) const {
    memory_pool_.Free(static_cast<State *>(particle));
}

int TiagoModel::NumActiveParticles() const {
    return memory_pool_.num_allocated();
}

int TiagoModel::indexOf(const vvs &v, const std::vector<std::string> &p) {
    for (int i = 0; i < v.size(); ++i) {
        if (v[i].size() != p.size())
            continue;
        bool allSame = true;
        for (size_t j = 0; j < p.size(); ++j) {
            if (v[i][j] != p[j]) {
                allSame = false;
                break;
            }
        }
        if (allSame)
            return i;
    }
    return -1;
}

double TiagoModel::Reward(const State &s, ACT_TYPE action) const {
    double reward = -1.0;
    const auto &actionName = actions_[action][0];
    if (actionName == "goto") {
        // 移动动作
        const auto &paraFrom = actions_[action][1];
        const auto &paraTo = actions_[action][2];
        auto cond1 = 1 << indexOf(propositions_, {"robot_at", paraFrom});
        if (cond1 != (s.state_id & cond1)) {
            reward = -INF;
        } else {
            reward = -1;
        }
    } else if (actionName == "pick") {
        const auto &paraTable = actions_[action][1];
        const auto &paraWaypoint = actions_[action][2];
        auto cond1 = 1 << indexOf(propositions_, {"robot_at", paraWaypoint});
        auto cond2 =
            1 << indexOf(propositions_, {"near", paraWaypoint, paraTable});
        auto cond3 = 1 << indexOf(propositions_, {"on_table", paraTable});
        auto cond4 = 1 << indexOf(propositions_, {"empty_hand"});
        auto cond = cond1 | cond2 | cond3 | cond4;
        if ((s.state_id & cond) != cond) {
            reward = -INF;
        } else {
            reward = -1;
        }
    } else {
        // place
        const auto &paraTable = actions_[action][1];
        const auto &paraWaypoint = actions_[action][2];
        auto cond1 = 1 << indexOf(propositions_, {"robot_at", paraWaypoint});
        auto cond2 =
            1 << indexOf(propositions_, {"near", paraWaypoint, paraTable});
        auto cond3 = 1 << indexOf(propositions_, {"in_hand"});
        auto cond4 = 1 << indexOf(propositions_, {"guest_not_near"});
        auto cond = cond1 | cond2 | cond3 | cond4;
        if ((s.state_id & cond) != cond) {
            reward = -INF;
        } else {
            reward = -1.0;
        }
    }

    auto goal1 = 1 << indexOf(propositions_, {"on_table", "table1"});
    auto goal2 = 1 << indexOf(propositions_, {"robot_at", "wp2"});
    auto goal = goal1 | goal2;
    // auto goal = 1 << indexOf(propositions_, {"in_hand"});
    if ((s.state_id & goal) == goal) {
        reward = INF;
    }
    return reward;
}

} // namespace despot
