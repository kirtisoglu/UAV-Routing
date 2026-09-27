"""
optimization.py
===============
Metaheuristic optimization algorithms for UAV routing.

The Optimizer class provides:
    - Iterated Local Search (ILS) with tabu perturbation
    - Simulated Annealing (SA) with configurable temperature schedules
    - Hill climbing (ascent runs)
    - Tilted runs (accept worse with fixed probability p)
    - Short bursts and variable-length short bursts

All methods are generators yielding State objects at each iteration,
allowing real-time tracking of convergence.
"""

from typing import Union, Callable, List, Any, Optional
import random
import math
from collections import Counter, defaultdict
from statistics import mean, median
from tqdm import tqdm

from uav_routing.local_search.accept import always_accept
from uav_routing.local_search.iterator import Iterator
from uav_routing.local_search.state import State

from uav_routing.local_search.proposal import random_flip_with_tabu, perturb_state


class Tally:
    """Per-operator counters + numerical distributions for an ILS run.

    Outcomes (mutually exclusive, one per attempt):
        accepted              -- move taken (SOCP feasible AND acceptance rule)
        worse                 -- SOCP feasible but rejected by acceptance rule
        infeas_cascade        -- TW pre-filter rejected the candidate (no SOCP call)
        infeas_cone           -- SOCP returned no solution (model infeasible)
        infeas_energy         -- SOCP returned solution but physical-energy
                                 post-check rejected it (cone slack catch)
        saturated             -- operator couldn't propose anything

    Numerical accumulators (per operator):
        energy_slack_accept   -- E_max - E_used for accepted tours (J)
        energy_overshoot_rej  -- E_phys - E_max for cone-slack rejects (J)
        tour_size_at_attempt  -- len(route) when operator was applied
        delta_obj_socp_feas   -- proposed.value - current.value for SOCP-feasible

    Read via Tally.summary() for per-operator rows, Tally.distributions()
    for histograms, or Tally.report() to print everything.
    """

    INFEAS_KINDS = ("infeas_cascade", "infeas_cone", "infeas_energy")
    OUTCOMES = ("accepted", "worse") + INFEAS_KINDS + ("saturated",)

    def __init__(self):
        self.attempts   = Counter()
        # Each outcome is its own counter
        for o in self.OUTCOMES:
            setattr(self, o, Counter())
        # Numerical distributions, per operator
        self.energy_slack_accept   = defaultdict(list)
        self.energy_overshoot_rej  = defaultdict(list)
        self.tour_size_at_attempt  = defaultdict(list)
        self.delta_obj_socp_feas   = defaultdict(list)

    def record_attempt(self, op: Optional[str], tour_size: Optional[int] = None) -> None:
        op = op or "unknown"
        self.attempts[op] += 1
        if tour_size is not None:
            self.tour_size_at_attempt[op].append(tour_size)

    def record(self, op: Optional[str], outcome: str) -> None:
        op = op or "unknown"
        if outcome not in self.OUTCOMES:
            raise ValueError(f"unknown outcome: {outcome!r}")
        getattr(self, outcome)[op] += 1

    def record_accept(self, op, energy_used, energy_max, delta_obj):
        op = op or "unknown"
        self.accepted[op] += 1
        if energy_used is not None and energy_max is not None:
            self.energy_slack_accept[op].append(energy_max - energy_used)
        if delta_obj is not None:
            self.delta_obj_socp_feas[op].append(delta_obj)

    def record_worse(self, op, energy_used, energy_max, delta_obj):
        op = op or "unknown"
        self.worse[op] += 1
        if delta_obj is not None:
            self.delta_obj_socp_feas[op].append(delta_obj)

    def record_infeas_energy(self, op, energy_phys, energy_max):
        op = op or "unknown"
        self.infeas_energy[op] += 1
        if energy_phys is not None and energy_max is not None:
            self.energy_overshoot_rej[op].append(energy_phys - energy_max)

    def total_infeas(self, op: str) -> int:
        return sum(getattr(self, k)[op] for k in self.INFEAS_KINDS)

    def summary(self) -> list:
        rows = []
        for op in sorted(self.attempts):
            a = self.attempts[op]
            row = {"operator": op, "attempts": a,
                   "accepted": self.accepted[op],
                   "worse":    self.worse[op],
                   "infeas_cascade": self.infeas_cascade[op],
                   "infeas_cone":    self.infeas_cone[op],
                   "infeas_energy":  self.infeas_energy[op],
                   "saturated":      self.saturated[op]}
            row["reject_rate"] = 1.0 - self.accepted[op] / max(1, a)
            socp_feas = self.accepted[op] + self.worse[op]
            row["SOCP_feas"] = socp_feas
            row["SOCP_feas_%"] = 100.0 * socp_feas / max(1, a)
            rows.append(row)
        return rows

    @staticmethod
    def _stats(xs):
        if not xs:
            return None
        s = sorted(xs)
        n = len(s)
        return {
            "n":      n,
            "min":    s[0],
            "p25":    s[int(0.25 * (n - 1))],
            "median": s[int(0.50 * (n - 1))],
            "p75":    s[int(0.75 * (n - 1))],
            "p90":    s[int(0.90 * (n - 1))],
            "max":    s[-1],
            "mean":   mean(s),
        }

    def distributions(self) -> dict:
        """Per-operator stats for all numerical accumulators."""
        result = {}
        for op in sorted(self.attempts):
            result[op] = {
                "tour_size_at_attempt":  self._stats(self.tour_size_at_attempt[op]),
                "energy_slack_accept":   self._stats(self.energy_slack_accept[op]),
                "energy_overshoot_rej":  self._stats(self.energy_overshoot_rej[op]),
                "delta_obj_socp_feas":   self._stats(self.delta_obj_socp_feas[op]),
            }
        return result

    def report(self, label: str = "", E_max: Optional[float] = None) -> str:
        """Print rich human-readable summary."""
        lines = []
        if label:
            lines.append(label)
        lines.append("")
        lines.append(f"{'operator':<22} {'att':>6} {'acc':>5} {'worse':>5} "
                     f"{'fcas':>5} {'fcon':>5} {'fen':>5} {'sat':>4} "
                     f"{'SOCPfeas%':>9}")
        lines.append("-" * 80)
        for r in self.summary():
            lines.append(f"{r['operator']:<22} {r['attempts']:>6} "
                         f"{r['accepted']:>5} {r['worse']:>5} "
                         f"{r['infeas_cascade']:>5} {r['infeas_cone']:>5} "
                         f"{r['infeas_energy']:>5} {r['saturated']:>4} "
                         f"{r['SOCP_feas_%']:>8.2f}%")
        lines.append("")
        lines.append("Legend:  fcas = cascade pre-filter rejected (no SOCP call)")
        lines.append("         fcon = SOCP model infeasible (no solution)")
        lines.append("         fen  = SOCP solution failed physical-energy post-check")
        lines.append("")

        # Distributions
        dists = self.distributions()
        for op, by_field in dists.items():
            for field_name, stats in by_field.items():
                if stats is None:
                    continue
                if field_name == "energy_slack_accept" and E_max:
                    pct = lambda v: 100.0 * v / E_max
                    lines.append(f"  {op}.{field_name} (% of E_max): "
                                 f"n={stats['n']} "
                                 f"median={pct(stats['median']):.3f}% "
                                 f"p25={pct(stats['p25']):.3f}% "
                                 f"p75={pct(stats['p75']):.3f}% "
                                 f"min={pct(stats['min']):.3f}%")
                elif field_name == "energy_overshoot_rej" and E_max:
                    pct = lambda v: 100.0 * v / E_max
                    lines.append(f"  {op}.{field_name} (% over E_max): "
                                 f"n={stats['n']} "
                                 f"median={pct(stats['median']):.3f}% "
                                 f"p75={pct(stats['p75']):.3f}% "
                                 f"p90={pct(stats['p90']):.3f}% "
                                 f"max={pct(stats['max']):.3f}%")
                else:
                    lines.append(f"  {op}.{field_name}: "
                                 f"n={stats['n']} "
                                 f"median={stats['median']:.3f} "
                                 f"p25={stats['p25']:.3f} "
                                 f"p75={stats['p75']:.3f}")
            lines.append("")
        return "\n".join(lines)


class Optimizer:
    """Metaheuristic optimizer for UAV tour improvement.

    Wraps a proposal function and an initial State, providing methods for
    ILS, SA, hill climbing, tilted runs, and short bursts. All methods are
    generators yielding State objects.

    The ``best_state`` and ``best_score`` properties track the best tour
    found during the current optimization run (reset at the start of each run).
    """
    
    def __init__(
        self,
        proposal: Callable[[State], State],
        initial_state: State,
        maximize: bool = True,
        ):
        """
        :param proposal: Function proposing the next state from the current state.
        :type proposal: Callable
        :param initial_state: Initial state of the optimizer.
        :type initial_state: State
        :param maximize: Boolean indicating whether to maximize or minimize the function.
            Defaults to True for maximize.
        :type maximize: bool, optional
        :param step_indexer: Name of the updater tracking the partitions step in the chain. If not
            implemented on the state the constructor creates and adds it. Defaults to "step".
        :type step_indexer: str, optional

        :return: An Optimizer object
        :rtype: Optimizer
        """
        self._initial_state = initial_state
        self._proposal = proposal
        self._score = lambda p: p.value
        self._maximize = maximize
        self._best_state = None
        self._best_score = None
        self.step = 1
   

    
    
    @property
    def best_state(self) -> State:
        """
        State object corresponding to best scoring tour observed over the current (or most
        recent) optimization run.

        :return: State object with the best score.
        :rtype: State
        """
        return self._best_state

    @property
    def best_score(self) -> Any:
        """
        Value of score metric corresponding to best scoring tour observed over the current (or most
        recent) optimization run.

        :return: Value of the best score.
        :rtype: Any
        """
        return self._best_score

    # TODO: no need for this. change it later.
    @property
    def score(self) -> Callable[[State], Any]:
        """
        The score function which is being optimized over.

        :return: The score function.
        :rtype: Callable[[State], Any]
        """
        return self._score

    def optimization_metric(self, State):
        """Return the optimization metric for a given state."""
        return State.value
    
    def _is_improvement(self, new_score: float, old_score: float) -> bool:
        """
        Helper function defining improvement comparison between scores.  
        Scores can be any comparable type.

        :param new_score: Score of proposed tour.
        :type new_score: float
        :param old_score: Score of previous tour.
        :type old_score: float

        :return: Whether the new score is an improvement over the old score.
        :rtype: bool
        """

        if self._maximize:
            return new_score >= old_score
        else:
            return new_score <= old_score



    def _tilted_acceptance_function(self, p: float) -> Callable[[State], bool]:
        """
        Function factory that binds and returns a tilted acceptance function.

        :param p: The probability of accepting a worse score.
        :type p: float

        :return: An acceptance function for tilted iterations.
        :rtype: Callable[[State], bool]
        """

        def tilted_acceptance_function(state):
            if state.parent is None:
                return True
            if state.solver.solution is None:
                return False

            state_score = self.score(state)
            prev_score = self.score(state.parent)

            if self._is_improvement(state_score, prev_score):
                return True
            else:
                return random.random() < p

        return tilted_acceptance_function


    def _simulated_annealing_acceptance_function(
        self, beta_function: Callable[[int], float], beta_magnitude: float
    ):
        """
        Function factory that binds and returns a simulated annealing acceptance function.

        :param beta_function: Function (f: t -> beta, where beta is in [0,1]) defining temperature
            over time.  f(t) = 0 the iterator is hot and every proposal is accepted.  At f(t) = 1 the
            iterator is cold and worse proposal have a low probability of being accepted relative to
            the magnitude of change in score.
        :type beta_function: Callable[[int], float]
        :param beta_magnitude: Scaling parameter for how much to weight changes in score.
        :type beta_magnitude: float

        :return: A acceptance function for simulated annealing runs.
        :rtype: Callable[[State], bool]
        """

        def simulated_annealing_acceptance_function(state):
            if state.parent is None:
                return True
            if state.solver.solution is None:
                return False

            score_delta = self.score(state) - self.score(state.parent)

            # Always accept improvements
            if (self._maximize and score_delta >= 0) or (not self._maximize and score_delta <= 0):
                return True

            # For worse moves: accept with probability exp(-beta * magnitude * |delta|/best)
            best = max(1.0, self._best_score if self._best_score else 1.0)
            normalized_delta = abs(score_delta) / best

            beta = beta_function(self.step)

            return random.random() < math.exp(-beta * beta_magnitude * normalized_delta)

        return simulated_annealing_acceptance_function

    @classmethod
    def jumpcycle_beta_function(
        cls, duration_hot: int, duration_cold: int
    ) -> Callable[[int], float]:
        """
        Class method that binds and return simple hot-cold cycle beta temperature function, where
        the iteration runs hot for some given duration and then cold for some duration, and repeats that
        cycle.

        :param duration_hot: Number of steps to run chain hot.
        :type duration_hot: int
        :param duration_cold: Number of steps to run chain cold.
        :type duration_cold: int

        :return: Beta function defining hot-cold cycle.
        :rtype: Callable[[int], float]
        """
        cycle_length = duration_hot + duration_cold

        def beta_function(step: int):
            time_in_cycle = step % cycle_length
            return float(time_in_cycle >= duration_hot)
        return beta_function


    @classmethod
    def linearcycle_beta_function(
        cls, duration_hot: int, duration_cooldown: int, duration_cold: int
    ) -> Callable[[int], float]:
        """
        Class method that binds and returns a simple linear hot-cool cycle beta temperature
        function, where the iteration runs hot for some given duration, cools down linearly for some
        duration, and then runs cold for some duration before warming up again and repeating.

        :param duration_hot: Number of steps to run iteration hot.
        :type duration_hot: int
        :param duration_cooldown: Number of steps needed to transition from hot to cold or
            vice-versa.
        :type duration_cooldown: int
        :param duration_cold: Number of steps to run iteration cold.
        :type duration_cold: int

        :return: Beta function defining linear hot-cool cycle.
        :rtype: Callable[[int], float]
        """
        cycle_length = duration_hot + 2 * duration_cooldown + duration_cold

        def beta_function(step: int):
            time_in_cycle = step % cycle_length
            if time_in_cycle < duration_hot:
                return 0
            elif time_in_cycle < duration_hot + duration_cooldown:
                return (time_in_cycle - duration_hot) / duration_cooldown
            elif time_in_cycle < cycle_length - duration_cooldown:
                return 1
            else:
                return (
                    1
                    - (time_in_cycle - cycle_length + duration_cooldown)
                    / duration_cooldown
                )
        return beta_function


    @classmethod
    def linear_jumpcycle_beta_function(
        cls, duration_hot: int, duration_cooldown, duration_cold: int
    ):
        """
        Class method that binds and returns a simple linear hot-cool cycle beta temperature
        function, where the iteration runs hot for some given duration, cools down linearly for some
        duration, and then runs cold for some duration before jumping back to hot and repeating.

        :param duration_hot: Number of steps to run iteration hot.
        :type duration_hot: int
        :param duration_cooldown: Number of steps needed to transition from hot to cold.
        :type duration_cooldown: int
        :param duration_cold: Number of steps to run iteration cold.
        :type duration_cold: int

        :return: Beta function defining linear hot-cool cycle.
        :rtype: Callable[[int], float]
        """
        cycle_length = duration_hot + duration_cooldown + duration_cold

        def beta_function(step: int):
            time_in_cycle = step % cycle_length
            if time_in_cycle < duration_hot:
                return 0
            elif time_in_cycle < duration_hot + duration_cooldown:
                return (time_in_cycle - duration_hot) / duration_cooldown
            else:
                return 1

        return beta_function

    @classmethod
    def logitcycle_beta_function(
        cls, duration_hot: int, duration_cooldown: int, duration_cold: int
    ) -> Callable[[int], float]:
        """
        Class method that binds and returns a logit hot-cool cycle beta temperature function, where
        the iteration runs hot for some given duration, cools down according to the logit function

        :math:`f(x) = (log(x/(1-x)) + 5)/10`

        for some duration, and then runs cold for some duration before warming up again
        using the :math:`1-f(x)` and repeating.

        :param duration_hot: Number of steps to run chain hot.
        :type duration_hot: int
        :param duration_cooldown: Number of steps needed to transition from hot to cold or
            vice-versa.
        :type duration_cooldown: int
        :param duration_cold: Number of steps to run chain cold.
        :type duration_cold: int
        """
        cycle_length = duration_hot + 2 * duration_cooldown + duration_cold

        # this will scale from 0 to 1 approximately
        logit = lambda x: (math.log(x / (1 - x)) + 5) / 10

        def beta_function(step: int):
            time_in_cycle = step % cycle_length
            if time_in_cycle <= duration_hot:
                return 0
            elif time_in_cycle < duration_hot + duration_cooldown:
                value = logit((time_in_cycle - duration_hot) / duration_cooldown)
                if value < 0:
                    return 0
                if value > 1:
                    return 1
                return value
            elif time_in_cycle <= cycle_length - duration_cooldown:
                return 1
            else:
                value = 1 - logit(
                    (time_in_cycle - cycle_length + duration_cooldown)
                    / duration_cooldown
                )
                if value < 0:
                    return 0
                if value > 1:
                    return 1
                return value

        return beta_function

    @classmethod
    def logit_jumpcycle_beta_function(
        cls, duration_hot: int, duration_cooldown: int, duration_cold: int
    ) -> Callable[[int], float]:
        """
        Class method that binds and returns a logit hot-cool cycle beta temperature function, where
        the iteration runs hot for some given duration, cools down according to the logit function

        :math:`f(x) = (log(x/(1-x)) + 5)/10`

        for some duration, and then runs cold for some duration before jumping back to hot and
        repeating.

        :param duration_hot: Number of steps to run iteration hot.
        :type duration_hot: int
        :param duration_cooldown: Number of steps needed to transition from hot to cold or
            vice-versa.
        :type duration_cooldown: int
        :param duration_cold: Number of steps to run iteration cold.
        :type duration_cold: int
        """
        cycle_length = duration_hot + duration_cooldown + duration_cold

        # this will scale from 0 to 1 approximately
        logit = lambda x: (math.log(x / (1 - x)) + 5) / 10

        def beta_function(step: int):
            time_in_cycle = step % cycle_length
            if time_in_cycle <= duration_hot:
                return 0
            elif time_in_cycle < duration_hot + duration_cooldown:
                value = logit((time_in_cycle - duration_hot) / duration_cooldown)
                if value < 0:
                    return 0
                if value > 1:
                    return 1
                return value
            else:
                return 1

        return beta_function

    
    
    def short_bursts(
        self,
        burst_length: int,
        num_bursts: int,
        accept: Callable[[State], bool] = always_accept,
        with_progress_bar: bool = False,
    ):
        """
        Performs a short burst run using the instance's score function. Each burst starts at the
        best performing state of the previous burst. If there's a tie, the later observed one is
        selected.

        :param burst_length: Number of steps to run within each burst.
        :type burst_length: int
        :param num_bursts: Number of bursts to perform.
        :type num_bursts: int
        :param accept: Function accepting or rejecting the proposed state. Defaults to always_accept()
        :type accept: Callable[[State], bool], optional
        :param with_progress_bar: Whether or not to draw tqdm progress bar. Defaults to False.
        :type with_progress_bar: bool, optional

        :return: State generator.
        :rtype: Generator[State]
        """
        if with_progress_bar:
            for state in tqdm(
                self.short_bursts(
                    burst_length, num_bursts, accept, with_progress_bar=False
                ),
                total=burst_length * num_bursts,
            ):
                yield state
            return

        self._best_state = self._initial_state
        self._best_score = self.score(self._best_state)

        for _ in range(num_bursts):
            iteration = Iterator(
                self._proposal, accept, self._best_state, burst_length
            )

            for state in iteration:
                yield state
                state_score = self.score(state)

                if self._is_improvement(state_score, self._best_score):
                    self._best_state = state
                    self._best_score = state_score

    def simulated_annealing(
        self,
        num_steps: int,
        beta_function: Callable[[int], float],
        beta_magnitude: float = 1,
        with_progress_bar: bool = False,
    ):
        """
        Performs simulated annealing with respect to the class instance's score function.

        :param num_steps: Number of steps to run for.
        :type num_steps: int
        :param beta_function: Function (f: t -> beta, where beta is in [0,1]) defining temperature
            over time.  f(t) = 0 the iteration is hot and every proposal is accepted. At f(t) = 1 the
            iteration is cold and worse proposal have a low probability of being accepted relative to
            the magnitude of change in score.
        :type beta_function: Callable[[int], float]
        :param beta_magnitude: Scaling parameter for how much to weight changes in score.
            Defaults to 1.
        :type beta_magnitude: float, optional
        :param with_progress_bar: Whether or not to draw tqdm progress bar. Defaults to False.
        :type with_progress_bar: bool, optional

        :return: State generator.
        :rtype: Generator[State]
        """
        iteration = Iterator(
            self._proposal,
            self._simulated_annealing_acceptance_function(
                beta_function, beta_magnitude
            ),
            self._initial_state,
            num_steps,
        )

        self._best_state = self._initial_state
        self._best_score = self.score(self._best_state)

        iteration_generator = tqdm(iteration) if with_progress_bar else iteration

        for state in iteration_generator:
            yield state
            self.step += 1
            state_score = self.score(state)
            if self._is_improvement(state_score, self._best_score):
                self._best_state = state
                self._best_score = state_score

    def tilted_short_bursts(
        self,
        burst_length: int,
        num_bursts: int,
        p: float,
        with_progress_bar: bool = False,
    ):
        """
        Performs a short burst run using the instance's score function. Each burst starts at the
        best performing tour of the previous burst. If there's a tie, the later observed one is
        selected. Within each burst a tilted acceptance function is used where better scoring tours
        are always accepted and worse scoring tours are accepted with probability `p`.

        :param burst_length: Number of steps to run within each burst.
        :type burst_length: int
        :param num_bursts: Number of bursts to perform.
        :type num_bursts: int
        :param p: The probability of accepting a tour with a worse score.
        :type p: float
        :param with_progress_bar: Whether or not to draw tqdm progress bar. Defaults to False.
        :type with_progress_bar: bool, optional


        :return: State generator.
        :rtype: Generator[State]
        """
        return self.short_bursts(
            burst_length,
            num_bursts,
            accept=self._tilted_acceptance_function(p),
            with_progress_bar=with_progress_bar,
        )

    # TODO: Maybe add a max_time variable so we don't run forever.
    def variable_length_short_bursts(
        self,
        num_steps: int,
        stuck_buffer: int,
        accept: Callable[[State], bool] = always_accept,
        with_progress_bar: bool = False,
    ):
        """
        Performs a short burst where the burst length is allowed to increase as it gets harder to
        find high scoring tours. The initial burst length is set to 2, and it is doubled each time
        there is no improvement over the passed number (`stuck_buffer`) of runs.

        :param num_steps: Number of steps to run for.
        :type num_steps: int
        :param stuck_buffer: How many bursts of a given length with no improvement to allow before
            increasing the burst length.
        :type stuck_buffer: int
        :param accept: Function accepting or rejecting the proposed state. Defaults to
        :type accept: Callable[[State], bool], optional
        :param with_progress_bar: Whether or not to draw tqdm progress bar. Defaults to False.
        :type with_progress_bar: bool, optional

        :return: State generator.
        :rtype: Generator[State]
        """
        if with_progress_bar:
            for state in tqdm(
                self.variable_length_short_bursts(
                    num_steps, stuck_buffer, accept, with_progress_bar=False
                ),
                total=num_steps,
            ):
                yield state
            return

        self._best_state = self._initial_state
        self._best_score = self.score(self._best_state)
        time_stuck = 0
        burst_length = 2
        i = 0

        while i < num_steps:
            iteration =Iterator(
                self._proposal, accept, self._best_state, burst_length
            )
            for state in iteration:
                yield state
                state_score = self.score(state)
                if self._is_improvement(state_score, self._best_score):
                    self._best_state = state
                    self._best_score = state_score
                    time_stuck = 0
                else:
                    time_stuck += 1

                i += 1
                if i >= num_steps:
                    break

            if time_stuck >= stuck_buffer * burst_length:
                burst_length *= 2
    
    def ascent_run(self, 
                   num_steps: int,
                   accept: Callable[[State], bool] = always_accept,
                   with_progress_bar: bool = False):
        """
        Performs an ascent run. An iterator where only better tours are accepted.
        
        :param num_steps: Number of steps to run for.
        :type num_steps: int
        :param with_progress_bar: Whether or not to draw tqdm progress bar. Defaults to False.
        :type with_progress_bar: bool, optional

        :return: State generator.
        :rtype: Generator[State]
        """
        iteration = Iterator(
            self._proposal,
            accept = accept,
            initial_state=self._initial_state,
            total_steps=num_steps,
        )

        self._best_state = self._initial_state
        self._best_score = self.score(self._best_state)

        iteration_generator = tqdm(iteration) if with_progress_bar else iteration

        for state in iteration_generator:
            yield state
            state_score = self.score(state)

            if self._is_improvement(state_score, self._best_score):
                self._best_state = state
                self._best_score = state_score
        return
    
    
    def tilted_run(self, num_steps: int, p: float, with_progress_bar: bool = False):
        """
        Performs a tilted run. An iterator where better tours are accepted and 
        worse tours with some probability `p`.

        :param num_steps: Number of steps to run for.
        :type num_steps: int
        :param p: The probability of accepting a tour with a worse score.
        :type p: float
        :param with_progress_bar: Whether or not to draw tqdm progress bar. Defaults to False.
        :type with_progress_bar: bool, optional

        :return: State generator.
        :rtype: Generator[State]
        """
        iteration = Iterator(
            self._proposal,
            self._tilted_acceptance_function(p),
            self._initial_state,
            num_steps,
        )

        self._best_state = self._initial_state
        self._best_score = self.score(self._best_state)

        iteration_generator = tqdm(iteration) if with_progress_bar else iteration

        for state in iteration_generator:
            yield state
            state_score = self.score(state)

            if self._is_improvement(state_score, self._best_score):
                self._best_state = state
                self._best_score = state_score
            
            
    def run_ils(
        self,
        total_steps: int = 1000,
        t_improve: int = 50,
        k_remove: int = 3,
        tabu_tenure: int = 20,
        with_progress_bar: bool = False,
        tally: bool = False,
    ):
        """
        Executes Iterated Local Search (ILS) as a generator.

        :param total_steps: Total number of steps to run.
        :param t_improve: Threshold of improvements before triggering a 'Shake'.
        :param k_remove: Number of nodes to remove during perturbation.
        :param tabu_tenure: How many iterations a removed node remains Tabu.
        :param tally: If True, collect per-operator attempt/outcome stats in
            ``self.tally`` (a :class:`Tally` instance). If False (default),
            ``self.tally`` is None and the hooks are no-ops.
        """
        # 0. Reset best state for this specific run
        self._best_state = self._initial_state
        self._best_score = self._initial_state.value

        # Initialize trackers
        self.tabu_list = {}  # {node_id: expiry_iteration}
        self.current_iteration = 0
        self.improvements_since_perturb = 0
        self.tally = Tally() if tally else None

        self.stagnation_counter = 0

        def ils_proposal_wrapper(current_state: State) -> State:
            self.current_iteration += 1

            # Purge expired tabu entries
            self.tabu_list = {node: expiry for node, expiry in self.tabu_list.items()
                              if expiry > self.current_iteration}

            # 1. Trigger Perturbation if stagnant
            if self.stagnation_counter >= t_improve:
                # Shake the current state
                perturbed_state, new_tabu_entries = perturb_state(
                    current_state, k_remove, self.current_iteration, tabu_tenure
                )
                self.tabu_list.update(new_tabu_entries)
                self.stagnation_counter = 0

                # Tag this state so the acceptance function knows to let it through
                perturbed_state.is_perturbation = True
                if self.tally is not None:
                    self.tally.record_attempt(perturbed_state.last_operator)
                return perturbed_state

            # 2. Standard Local Move
            proposed = random_flip_with_tabu(current_state, set(self.tabu_list.keys()))
            if self.tally is not None:
                self.tally.record_attempt(proposed.last_operator)
            return proposed

        def ils_accept(proposed_state: State) -> bool:
            if proposed_state.solver is None or proposed_state.solver.solution is None:
                if self.tally is not None:
                    self.tally.record(proposed_state.last_operator, "infeasible")
                return False

            # ALWAYS accept a perturbation to allow the chain to move basins
            if getattr(proposed_state, 'is_perturbation', False):
                if self.tally is not None:
                    self.tally.record(proposed_state.last_operator, "accepted")
                return True

            proposed_score = proposed_state.value
            parent_score = proposed_state.parent.value if proposed_state.parent else -float('inf')

            if self._is_improvement(proposed_score, parent_score):
                if self._is_improvement(proposed_score, self._best_score):
                    self._best_state = proposed_state
                    self._best_score = proposed_score
                self.stagnation_counter = 0  # Local improvement resets stagnation
                if self.tally is not None:
                    self.tally.record(proposed_state.last_operator, "accepted")
                return True

            self.stagnation_counter += 1  # No improvement
            if self.tally is not None:
                self.tally.record(proposed_state.last_operator, "worse")
            return False
        # 3. Initialize the Iterator
        chain = Iterator(
            proposal=ils_proposal_wrapper,
            accept=ils_accept,
            initial_state=self._initial_state,
            total_steps=total_steps
        )

        # 4. Yield states to support the 'for i, state in enumerate(...)' pattern
        iterator_loop = chain.with_progress_bar() if with_progress_bar else chain
        
        for state in iterator_loop:
            yield state