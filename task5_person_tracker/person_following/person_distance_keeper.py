#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Backward-compatible wrapper.

Use person_goal_publisher.py instead.
"""

from person_goal_publisher import PersonGoalPublisher


if __name__ == "__main__":
    node = PersonGoalPublisher()
    node.run()
