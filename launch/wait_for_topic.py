#!/usr/bin/env python3
"""
WaitForTopic Launch Action

A custom launch action that blocks until a specified ROS2 topic becomes available.
This enables true event-driven launch sequencing where dependent nodes only start
after their required topics exist.

Based on the design from ros2/launch_ros#434.

Usage in launch file:
    from launch.wait_for_topic import WaitForTopic

    ld = LaunchDescription([
        some_node,
        WaitForTopic('/clock'),
        dependent_node,  # Only starts after /clock is available
    ])

Author: Alexander Shultis
Date: December 2025
"""

import asyncio
import logging
import signal
from typing import Optional

from launch import Action
from launch.launch_context import LaunchContext


class WaitForTopic(Action):
    """
    Launch action that blocks until a ROS2 topic becomes available.

    This action polls the ROS2 topic list until the specified topic appears,
    then completes and allows subsequent actions to proceed.
    """

    def __init__(
        self,
        topic_name: str,
        timeout: float = 60.0,
        poll_interval: float = 0.5,
        **kwargs
    ):
        """
        Initialize WaitForTopic action.

        Args:
            topic_name: The name of the topic to wait for (e.g., '/clock')
            timeout: Maximum time to wait in seconds (default 60.0)
            poll_interval: Time between topic list checks in seconds (default 0.5)
        """
        super().__init__(**kwargs)
        self.topic_name = topic_name
        self.timeout = timeout
        self.poll_interval = poll_interval
        self.logger = logging.getLogger(f'WaitForTopic({topic_name})')
        self._completion_future: Optional[asyncio.Future] = None
        self._node = None
        self._shutdown_requested = False

    def execute(self, context: LaunchContext):
        """Execute the action - wait for the topic to become available."""
        # Import rclpy here to avoid import errors before ROS is initialized
        import rclpy
        from rclpy.node import Node

        # Initialize rclpy if not already done
        if not rclpy.ok():
            rclpy.init()

        # Create a temporary node for topic introspection
        self._node = Node(f'_wait_for_topic_{self.topic_name.replace("/", "_")}')

        # Create completion future
        self._completion_future = context.asyncio_loop.create_future()

        # Set up signal handler for graceful shutdown
        def signal_handler(sig, frame):
            self._shutdown_requested = True
            if self._completion_future and not self._completion_future.done():
                self._completion_future.set_result(None)

        original_handler = signal.signal(signal.SIGINT, signal_handler)

        # Start the async polling task
        async def poll_for_topic():
            """Periodically check if the topic is available."""
            elapsed = 0.0
            self.logger.info(f'Waiting for topic {self.topic_name}...')

            while not self._shutdown_requested and elapsed < self.timeout:
                # Get list of available topics
                topic_names_and_types = self._node.get_topic_names_and_types()
                topic_names = [name for name, _ in topic_names_and_types]

                if self.topic_name in topic_names:
                    self.logger.info(f'Topic {self.topic_name} is now available!')
                    break

                await asyncio.sleep(self.poll_interval)
                elapsed += self.poll_interval

            if elapsed >= self.timeout:
                self.logger.warning(f'Timeout waiting for topic {self.topic_name}')

            # Cleanup
            self._node.destroy_node()
            signal.signal(signal.SIGINT, original_handler)

            if not self._completion_future.done():
                self._completion_future.set_result(None)

        # Schedule the polling task
        context.asyncio_loop.create_task(poll_for_topic())

        # Register the completion future with the launch context
        context.add_completion_future(self._completion_future)

        return None

    def get_asyncio_future(self):
        """Return the asyncio future for the launch system to wait on."""
        return self._completion_future


class WaitForTopics(Action):
    """
    Launch action that blocks until ALL specified ROS2 topics become available.

    Similar to WaitForTopic but waits for multiple topics at once.
    """

    def __init__(
        self,
        topic_names: list,
        timeout: float = 60.0,
        poll_interval: float = 0.5,
        **kwargs
    ):
        """
        Initialize WaitForTopics action.

        Args:
            topic_names: List of topic names to wait for
            timeout: Maximum time to wait in seconds (default 60.0)
            poll_interval: Time between topic list checks in seconds (default 0.5)
        """
        super().__init__(**kwargs)
        self.topic_names = set(topic_names)
        self.timeout = timeout
        self.poll_interval = poll_interval
        topics_str = ', '.join(topic_names[:3])
        if len(topic_names) > 3:
            topics_str += f'... (+{len(topic_names)-3} more)'
        self.logger = logging.getLogger(f'WaitForTopics({topics_str})')
        self._completion_future: Optional[asyncio.Future] = None
        self._node = None
        self._shutdown_requested = False

    def execute(self, context: LaunchContext):
        """Execute the action - wait for all topics to become available."""
        import rclpy
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()

        self._node = Node('_wait_for_topics_node')
        self._completion_future = context.asyncio_loop.create_future()

        def signal_handler(sig, frame):
            self._shutdown_requested = True
            if self._completion_future and not self._completion_future.done():
                self._completion_future.set_result(None)

        original_handler = signal.signal(signal.SIGINT, signal_handler)

        async def poll_for_topics():
            """Periodically check if all topics are available."""
            elapsed = 0.0
            remaining = set(self.topic_names)
            self.logger.info(f'Waiting for {len(remaining)} topics...')

            while not self._shutdown_requested and elapsed < self.timeout and remaining:
                topic_names_and_types = self._node.get_topic_names_and_types()
                available = {name for name, _ in topic_names_and_types}

                newly_found = remaining & available
                if newly_found:
                    for topic in newly_found:
                        self.logger.info(f'Topic {topic} is now available')
                    remaining -= newly_found

                if not remaining:
                    self.logger.info('All topics are now available!')
                    break

                await asyncio.sleep(self.poll_interval)
                elapsed += self.poll_interval

            if elapsed >= self.timeout and remaining:
                self.logger.warning(f'Timeout waiting for topics: {remaining}')

            self._node.destroy_node()
            signal.signal(signal.SIGINT, original_handler)

            if not self._completion_future.done():
                self._completion_future.set_result(None)

        context.asyncio_loop.create_task(poll_for_topics())
        context.add_completion_future(self._completion_future)

        return None

    def get_asyncio_future(self):
        """Return the asyncio future for the launch system to wait on."""
        return self._completion_future
