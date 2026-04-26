"""Mission finite state machine.

Holds only the state name and timing; the actual logic lives in the main
controller. Keeping the FSM dumb makes it trivial to debug from the log.
"""
import math


class State:
    INIT = "INIT"
    INIT_SCAN = "INIT_SCAN"
    EXPLORE_BLUE = "EXPLORE_BLUE"
    GO_BLUE = "GO_BLUE"
    EXPLORE_YELLOW = "EXPLORE_YELLOW"
    GO_YELLOW = "GO_YELLOW"
    RECOVERY = "RECOVERY"
    DONE = "DONE"

    GOALS = {
        EXPLORE_BLUE: "blue", GO_BLUE: "blue",
        EXPLORE_YELLOW: "yellow", GO_YELLOW: "yellow",
    }


class MissionFSM:
    def __init__(self, log_fn=print):
        self.state = State.INIT
        self.t_enter = 0.0
        self.recovery_chain = 0
        self.prev_state = None
        self.log = log_fn

    def transition(self, new_state, sim_time):
        self.log(f"[fsm] {self.state} -> {new_state} @ t={sim_time:.2f}s")
        if new_state == State.RECOVERY:
            self.prev_state = self.state
            self.recovery_chain += 1
        if new_state != State.RECOVERY and self.state == State.RECOVERY:
            # leaving recovery — counter resets only after a successful spell
            # of normal driving (handled by the caller via reset_recovery_chain)
            pass
        self.state = new_state
        self.t_enter = sim_time

    def time_in_state(self, sim_time):
        return sim_time - self.t_enter

    def reset_recovery_chain(self):
        self.recovery_chain = 0

    def target_color(self):
        return State.GOALS.get(self.state)
