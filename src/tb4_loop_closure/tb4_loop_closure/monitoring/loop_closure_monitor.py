#!/usr/bin/env python3

"""
Loop closure monitoring utilities.

This module contains no ROS 2 Node logic. It only compares the loop-closure
links contained in consecutive RTAB-Map pose graphs and reports newly added
links.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable


@dataclass(frozen=True, order=True)
class LoopClosureEvent:
    """
    Represents one newly detected loop closure.

    Attributes
    ----------
    from_id:
        One endpoint of the loop-closure edge.

    to_id:
        The other endpoint of the loop-closure edge.

    link_type:
        RTAB-Map link type. Loop closures normally use type 1.
    """

    from_id: int
    to_id: int
    link_type: int = 1


class LoopClosureMonitor:
    """
    Detect newly added loop-closure links in an RTAB-Map pose graph.

    The first graph received by this class is treated as the initial state.
    Existing loop closures in that first graph are recorded but are not
    reported as new events.

    Notes
    -----
    Loop-closure edges are normalized as:

        (min(from_id, to_id), max(from_id, to_id))

    Therefore, edges 49 -> 78 and 78 -> 49 are treated as the same edge.
    """

    LOOP_CLOSURE_TYPE = 1

    def __init__(self) -> None:
        self._initialized = False
        self._previous_loop_links: set[tuple[int, int]] = set()

    @property
    def initialized(self) -> bool:
        """Return whether the first graph has already been recorded."""
        return self._initialized

    @property
    def known_loop_count(self) -> int:
        """Return the number of loop closures currently known."""
        return len(self._previous_loop_links)

    def reset(self) -> None:
        """
        Clear all monitoring state.

        After reset(), the next graph will once again be treated as the
        initial graph and will not produce new-loop events.
        """
        self._initialized = False
        self._previous_loop_links.clear()

    def detect_new_links(self, graph: Any) -> list[LoopClosureEvent]:
        """
        Detect loop-closure links that were not present in the previous graph.

        Parameters
        ----------
        graph:
            An RTAB-Map graph object containing a ``links`` attribute.

            Normally this is an ``rtabmap_msgs/msg/Graph`` message.

        Returns
        -------
        list[LoopClosureEvent]
            Newly added loop closures.

            The first call always returns an empty list because it is used
            to initialize the monitor.
        """
        links = self._get_links(graph)
        current_loop_links = self._collect_loop_links(links)

        # First graph establishes the initial state.
        if not self._initialized:
            self._previous_loop_links = current_loop_links
            self._initialized = True
            return []

        new_links = current_loop_links - self._previous_loop_links

        # Replace the previous state with the newest graph state.
        self._previous_loop_links = current_loop_links

        return [
            LoopClosureEvent(
                from_id=from_id,
                to_id=to_id,
                link_type=self.LOOP_CLOSURE_TYPE,
            )
            for from_id, to_id in sorted(new_links)
        ]

    def _collect_loop_links(
        self,
        links: Iterable[Any],
    ) -> set[tuple[int, int]]:
        """
        Extract normalized type-1 links from an iterable of RTAB-Map links.
        """
        loop_links: set[tuple[int, int]] = set()

        for link in links:
            link_type = self._read_integer_field(link, "type")

            if link_type != self.LOOP_CLOSURE_TYPE:
                continue

            from_id = self._read_integer_field(link, "from_id")
            to_id = self._read_integer_field(link, "to_id")

            # Ignore invalid self-loop edges.
            if from_id == to_id:
                continue

            normalized_edge = self._normalize_edge(from_id, to_id)
            loop_links.add(normalized_edge)

        return loop_links

    @staticmethod
    def _get_links(graph: Any) -> Iterable[Any]:
        """
        Retrieve the ``links`` field from a graph-like object.
        """
        if graph is None:
            raise ValueError("graph cannot be None")

        if not hasattr(graph, "links"):
            raise AttributeError(
                "The provided graph object has no 'links' attribute."
            )

        links = graph.links

        if links is None:
            return []

        return links

    @staticmethod
    def _normalize_edge(
        from_id: int,
        to_id: int,
    ) -> tuple[int, int]:
        """
        Normalize edge direction so reversed edges compare equally.
        """
        return min(from_id, to_id), max(from_id, to_id)

    @staticmethod
    def _read_integer_field(
        message: Any,
        field_name: str,
    ) -> int:
        """
        Read and validate an integer-like field from a ROS message.
        """
        if not hasattr(message, field_name):
            raise AttributeError(
                f"Link object has no '{field_name}' attribute."
            )

        value = getattr(message, field_name)

        try:
            return int(value)
        except (TypeError, ValueError) as error:
            raise ValueError(
                f"Link field '{field_name}' must be integer-like, "
                f"but received {value!r}."
            ) from error