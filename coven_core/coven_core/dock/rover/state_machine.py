"""
state_machine.py - Module FSM (Finite State Machine)

Implements the COVEN module lifecycle using an explicit transition table pattern.
This follows the state machine best practices from YASMIN/SMACH but without
the library overhead - our FSM is simple enough for a clean typed implementation.

States:
    BOOT → WAIT_VERIFY → NORMAL ⟷ FIELD_OPS
                ↓
            REJECTED

The FSM validates all transitions and provides hooks for state change events.

References:
- YASMIN: https://github.com/uleroboticsgroup/yasmin
- ROS2 Lifecycle: https://design.ros2.org/articles/node_lifecycle.html

Author: Alexander Shultis
Date: December 2025
"""

import logging
from enum import Enum, auto
from typing import Callable, Set, Dict
from dataclasses import dataclass

logger = logging.getLogger(__name__)


class ModuleState(Enum):
    """FSM states for a COVEN module."""

    BOOT = auto()          # Initial state, waiting for IDENTIFY
    WAIT_VERIFY = auto()   # Identified, waiting for verification
    NORMAL = auto()        # Verified and enabled, ready for tasks
    FIELD_OPS = auto()     # Executing a task
    REJECTED = auto()      # Failed verification
    DISCONNECTED = auto()  # Lost connection to dock


@dataclass
class TransitionResult:
    """Result of a state transition attempt."""

    success: bool
    from_state: ModuleState
    to_state: ModuleState
    reason: str = ""


# Type alias for transition callbacks
TransitionCallback = Callable[[ModuleState, ModuleState], None]


class ModuleStateMachine:
    """
    Finite State Machine for COVEN module lifecycle.

    Uses an explicit transition table to validate all state changes.
    Supports callbacks for state entry/exit and transition events.

    Example:
        fsm = ModuleStateMachine()
        fsm.on_transition(lambda from_s, to_s: print(f"{from_s} -> {to_s}"))

        fsm.transition_to(ModuleState.WAIT_VERIFY)  # Valid from BOOT
        fsm.transition_to(ModuleState.NORMAL)        # Valid from WAIT_VERIFY
        fsm.transition_to(ModuleState.BOOT)          # Invalid! Raises or returns failure
    """

    # Valid state transitions: {from_state: {valid_to_states}}
    TRANSITIONS: Dict[ModuleState, Set[ModuleState]] = {
        ModuleState.BOOT: {
            ModuleState.WAIT_VERIFY,  # After IDENTIFY_REQ
        },
        ModuleState.WAIT_VERIFY: {
            ModuleState.NORMAL,       # After verification + power enable
            ModuleState.REJECTED,     # Failed verification
        },
        ModuleState.NORMAL: {
            ModuleState.FIELD_OPS,    # Task assigned
            ModuleState.DISCONNECTED, # Lost heartbeat
        },
        ModuleState.FIELD_OPS: {
            ModuleState.NORMAL,       # Task complete or cancelled
            ModuleState.DISCONNECTED, # Lost during task
        },
        ModuleState.REJECTED: {
            ModuleState.BOOT,         # Reset for retry
        },
        ModuleState.DISCONNECTED: {
            ModuleState.BOOT,         # Reconnection attempt
        },
    }

    def __init__(self, initial_state: ModuleState = ModuleState.BOOT):
        """
        Initialize the state machine.

        Args:
            initial_state: Starting state (default: BOOT)
        """
        self._state = initial_state
        self._callbacks: list[TransitionCallback] = []
        self._entry_callbacks: Dict[ModuleState, list[Callable[[], None]]] = {
            s: [] for s in ModuleState
        }
        self._exit_callbacks: Dict[ModuleState, list[Callable[[], None]]] = {
            s: [] for s in ModuleState
        }

    @property
    def state(self) -> ModuleState:
        """Current state (read-only)."""
        return self._state

    def can_transition_to(self, target: ModuleState) -> bool:
        """
        Check if transition to target state is valid.

        Args:
            target: Proposed target state

        Returns:
            True if transition is allowed
        """
        valid_targets = self.TRANSITIONS.get(self._state, set())
        return target in valid_targets

    def transition_to(self, target: ModuleState, validate: bool = True) -> TransitionResult:
        """
        Attempt to transition to a new state.

        Args:
            target: Target state
            validate: If True (default), reject invalid transitions.
                      If False, allow any transition (for testing/recovery).

        Returns:
            TransitionResult with success status and details
        """
        from_state = self._state

        # Validate transition
        if validate and not self.can_transition_to(target):
            valid = self.TRANSITIONS.get(self._state, set())
            reason = (
                f"Invalid transition: {self._state.name} -> {target.name}. "
                f"Valid targets: {[s.name for s in valid]}"
            )
            logger.warning(reason)
            return TransitionResult(
                success=False,
                from_state=from_state,
                to_state=self._state,
                reason=reason
            )

        # Execute exit callbacks for old state
        for callback in self._exit_callbacks[self._state]:
            try:
                callback()
            except Exception as e:
                logger.error(f"Exit callback failed for {self._state.name}: {e}")

        # Update state
        self._state = target
        logger.info(f"State transition: {from_state.name} -> {target.name}")

        # Execute entry callbacks for new state
        for callback in self._entry_callbacks[target]:
            try:
                callback()
            except Exception as e:
                logger.error(f"Entry callback failed for {target.name}: {e}")

        # Execute general transition callbacks
        for callback in self._callbacks:
            try:
                callback(from_state, target)
            except Exception as e:
                logger.error(f"Transition callback failed: {e}")

        return TransitionResult(
            success=True,
            from_state=from_state,
            to_state=target,
            reason=""
        )

    def force_state(self, target: ModuleState):
        """
        Force state without validation (for error recovery).

        Use sparingly - prefer transition_to() for normal operation.

        Args:
            target: Target state to force
        """
        logger.warning(f"Forcing state: {self._state.name} -> {target.name}")
        return self.transition_to(target, validate=False)

    def on_transition(self, callback: TransitionCallback):
        """
        Register a callback for all state transitions.

        Args:
            callback: Function called with (from_state, to_state)
        """
        self._callbacks.append(callback)

    def on_enter(self, state: ModuleState, callback: Callable[[], None]):
        """
        Register a callback for entering a specific state.

        Args:
            state: State to watch
            callback: Function called when entering state
        """
        self._entry_callbacks[state].append(callback)

    def on_exit(self, state: ModuleState, callback: Callable[[], None]):
        """
        Register a callback for exiting a specific state.

        Args:
            state: State to watch
            callback: Function called when leaving state
        """
        self._exit_callbacks[state].append(callback)

    def reset(self):
        """Reset to initial BOOT state."""
        self._state = ModuleState.BOOT
        logger.info("State machine reset to BOOT")

    def __repr__(self) -> str:
        return f"ModuleStateMachine(state={self._state.name})"
