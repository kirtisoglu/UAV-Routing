
from typing import Union, Callable, List, Any
import random
import math
from tqdm import tqdm

from uav_routing.local_search.accept import always_accept
from uav_routing.local_search.iterator import Iterator
from uav_routing.local_search.state import State

from uav_routing.local_search.proposal import random_flip_with_tabu, perturb_state


class Optimizer:
    """
    Optimizer represents the class of algorithms that optimize tours with respect to 
    a single metric. An instance of this class encapsulates the following state
    information:
        * the dual graph and updaters via the initial partition,
        * the metric over which to optimize,
        * and whether or not to seek maximal or minimal values of the metric.

    The SingleMetricOptimizer class implements the following common methods of optimization:
        * Short Bursts
        * Simulated Annealing
        * Tilted Runs

    Both during and after a optimization run, the class properties `best_part` and `best_score`
    represent the optimal state / corresponding score value observed.  Note that these
    properties do NOT persist across multiple optimization runs, as they are reset each time an
    optimization run is invoked.
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
            if state.solver.solution == None:
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
            if state.solver.solution == None:
                return False
            
            score_delta = self.score(state) - self.score(state.parent)
            
            # Normalization: Divide delta by best_score so change is a % 
            # We use max(1, best) to avoid division by zero
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
            state_score = self.score(state)
            if self._is_improvement(state_score, self._best_score):
                self._best_state = state
                self._best_score = state_score
                self.step += 1

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
        with_progress_bar: bool = True
    ):
        """
        Executes Iterated Local Search (ILS) as a generator.
        
        :param total_steps: Total number of steps to run.
        :param t_improve: Threshold of improvements before triggering a 'Shake'.
        :param k_remove: Number of nodes to remove during perturbation.
        :param tabu_tenure: How many iterations a removed node remains Tabu.
        """
        # 0. Reset best state for this specific run
        self._best_state = self._initial_state
        self._best_score = self._initial_state.value
        
        # Initialize trackers
        self.tabu_list = {}  # {node_id: expiry_iteration}
        self.current_iteration = 0
        self.improvements_since_perturb = 0

        # 1. Define the Proposal Wrapper (Closure)
        def ils_proposal_wrapper(current_state: State) -> State:
            # Update internal iteration count
            self.current_iteration += 1
            
            # Clean expired tabu entries
            self.tabu_list = {node: expiry for node, expiry in self.tabu_list.items() 
                            if expiry > self.current_iteration}
            
            # Determine if we need to Perturb (The Shake)
            if self.improvements_since_perturb >= t_improve:
                # Trigger perturbation
                perturbed_state, new_tabu_entries = perturb_state(
                    current_state, k_remove, self.current_iteration, tabu_tenure
                )
                self.tabu_list.update(new_tabu_entries)
                self.improvements_since_perturb = 0 # Reset counter after shake
                return perturbed_state
            
            # Otherwise, perform a standard local move
            return random_flip_with_tabu(current_state, set(self.tabu_list.keys()))

        # 2. Define the Acceptance Function
        def ils_accept(proposed_state: State) -> bool:
            # Reject if the SOCP solver failed or the graph is invalid
            if proposed_state.solver is None or proposed_state.solver.solution is None:
                return False
            
            proposed_score = proposed_state.value
            # Compare to parent to determine if we move the chain
            parent_score = proposed_state.parent.value if proposed_state.parent else -float('inf')
            
            # Hill-climbing logic
            if self._is_improvement(proposed_score, parent_score):
                # Update global best tracking
                if self._is_improvement(proposed_score, self._best_score):
                    self._best_state = proposed_state
                    self._best_score = proposed_score
                    # Increment counter to eventually trigger a Shake
                    self.improvements_since_perturb += 1
                return True
            
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
            
   
    
    def run_ils(
        self,
        total_steps: int = 1000,
        t_improve: int = 50,
        k_remove: int = 3,
        tabu_tenure: int = 20,
        with_progress_bar: bool = True
    ):
        """
        Executes Iterated Local Search (ILS) as a generator.
        
        :param total_steps: Total number of steps to run.
        :param t_improve: Threshold of improvements before triggering a 'Shake'.
        :param k_remove: Number of nodes to remove during perturbation.
        :param tabu_tenure: How many iterations a removed node remains Tabu.
        """
        # 0. Reset best state for this specific run
        self._best_state = self._initial_state
        self._best_score = self._initial_state.value
        
        # Initialize trackers
        self.tabu_list = {}  # {node_id: expiry_iteration}
        self.current_iteration = 0
        self.improvements_since_perturb = 0
        
        self.stagnation_counter = 0 

        def ils_proposal_wrapper(current_state: State) -> State:
            self.current_iteration += 1
            
            # 1. Trigger Perturbation if stagnant
            if self.stagnation_counter >= t_improve:
                # Shake the BEST state, not the current state
                perturbed_state, new_tabu_entries = perturb_state(
                    self._best_state, k_remove, self.current_iteration, tabu_tenure
                )
                self.tabu_list.update(new_tabu_entries)
                self.stagnation_counter = 0 
                
                # Tag this state so the acceptance function knows to let it through
                perturbed_state.is_perturbation = True 
                return perturbed_state
            
            # 2. Standard Local Move
            return random_flip_with_tabu(current_state, set(self.tabu_list.keys()))

        def ils_accept(proposed_state: State) -> bool:
            if proposed_state.solver is None or proposed_state.solver.solution is None:
                return False
            
            # ALWAYS accept a perturbation to allow the chain to move basins
            if getattr(proposed_state, 'is_perturbation', False):
                return True

            proposed_score = proposed_state.value
            parent_score = proposed_state.parent.value if proposed_state.parent else -float('inf')
            
            if self._is_improvement(proposed_score, parent_score):
                if self._is_improvement(proposed_score, self._best_score):
                    self._best_state = proposed_state
                    self._best_score = proposed_score
                    self.stagnation_counter = 0 # Found a new global best
                return True
            
            self.stagnation_counter += 1 # Didn't improve
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