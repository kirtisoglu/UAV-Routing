"""
This module provides the Iterator class, which is designed to facilitate the creation
and iteration of states in the context of UAV routing plans. It allows for the exploration 
of different routes based on specified acceptance criteria.

Key Components:

- Iterator: The main class used for creating and iterating over states.

Usage:
The primary use of this module is to create an instance of Iterator with appropriate
parameters like proposal function, acceptance function, and initial state, and then 
to iterate through the states of the Iterator, yielding a new proposal at each step.

Dependencies:

- typing: Used for type hints.

Last Updated: 
"""

from typing import Union, Iterable, Callable, Optional
from .state import State


class Iterator:
    """
    A class that creates an iterator for iterating over the states
    of a run in a UAV routing analysis context.

    It allows for the generation of a sequence of states of tours.

    Example usage:

    .. code-block:: python

        chain = Iterator(proposal, accept, initial_state, total_steps)
        for state in chain:
            # Do whatever you want - print output, compute scores, ...
    """

    def __init__(
        self,
        proposal: Callable,
        accept: Callable,
        initial_state: State,
        total_steps: int,
    ) -> None:
        """
        :param proposal: Function proposing the next state from the current state.
        :type proposal: Callable
        :param accept: Function accepting or rejecting the proposed state. In the most basic
            use case, this always returns ``True``. But if the user wanted to use a
            Metropolis-Hastings acceptance rule, this is where you would implement it.
        :type accept: Callable
        :param initial_state: Initial :class:`gerrychain.partition.Partition` class.
        :type initial_state: Partition
        :param total_steps: Number of steps to run.
        :type total_steps: int

        :returns: None
        """

        self.proposal = proposal
        self.accept = accept
        self.total_steps = total_steps
        self.initial_state = initial_state
        self.state = initial_state



    def __iter__(self) -> "Iterator":
        """
        Resets the iterator.

        This method is called when an iterator is required for a container. It sets the
        counter to 0 and resets the state to the initial state.

        :returns: Returns itself as an iterator object.
        :rtype: MarkovChain
        """
        self.counter = 0
        self.state = self.initial_state
        return self

    def __next__(self) -> Optional[State]:
        """
        Advances the Iterator to the next state.

        This method is called to get the next item in the iteration.
        It proposes the next state and moves to it if accepted by the
        acceptance function. If the total number of steps has been
        reached, it raises a StopIteration exception.

        :returns: The next state of the chain.
        :rtype: Optional[State]

        :raises StopIteration: If the total number of steps has been reached.
        """
        if self.counter == 0:
            self.counter += 1
            return self.state

        while self.counter < self.total_steps:
            proposed_next_state = self.proposal(self.state) # propose here
            # Erase the parent of the parent, to avoid memory leak
            if self.state is not None:
                self.state.parent = None

            if self.accept(proposed_next_state):
                self.state = proposed_next_state
            self.counter += 1
            return self.state
        raise StopIteration

    def __len__(self) -> int:
        """
        Returns the total number of steps in the Iterator.

        :returns: The total number of steps in the Iterator.
        :rtype: int
        """
        return self.total_steps

    def __repr__(self) -> str:
        return "<Iterator [{} steps]>".format(len(self))

    def with_progress_bar(self):
        """
        Wraps the Markov chain in a tqdm progress bar.

        Useful for long-running Markov chains where you want to keep track
        of the progress. Requires the `tqdm` package to be installed.

        :returns: A tqdm-wrapped Markov chain.
        """
        from tqdm.auto import tqdm

        return tqdm(self)