# Agent type
* **Problem-Solving agent**

# Task Environment PEAS Description

## Performance Measure
* **Fastest trajectory**

## Environment
* **Fully observable**
* **Single-agent**
* **Deterministic**
* **Episodic**
* **Static**
* **Discrete**

# Search Problem Formulation

## State Space
Each state is of the form (x, y), where x and y are the x-axis and y-axis integer coordinates (not related to real measurements), respectively.
The state space should be represented as a graph.

## Initial State
(x, y)

## Goal States
(x', y')

## Actions
From (x, y) goto:
* (x+1, y)
* (x-1, y)
* (x, y+1)
* (x, y-1)
Note that for some coordinates, some actions are not available.

## Transition Model
*RESULT*(s, a) = a +- {(+-1, 0) v (0, +-1)}

## Action Cost Function
*ACTION-COST*(s, a, s') = 1



