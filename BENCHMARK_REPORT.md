# Benchmark Report: Collision Checking Optimization

## Overview
This report compares the performance of the collision checking algorithm between the `main` branch and the `arjo/feat/more_efficient_checks` branch. The benchmarks were conducted using the `agent_scaling` suite, which measures the time taken to process plans as the number of agents increases.

## Results Summary

| Agent Count | Main Branch (ms) | Optimized Branch (ms) | Speedup |
| :--- | :--- | :--- | :--- |
| 10 | 2.08 | 0.35 | **~5.9x** |
| 50 | 59.62 | 2.58 | **~23.1x** |
| 100 | 244.89 | 5.48 | **~44.7x** |
| 200 | 963.66 | 11.37 | **~84.7x** |
| 500 | ~6,000.00* | 30.54 | **~196.5x** |

*\*Estimated based on incomplete run on main.*

## Analysis
The optimized branch (`arjo/feat/more_efficient_checks`) demonstrates significantly better scaling characteristics:

1.  **Linear Scaling:** While the `main` branch appears to scale quadratically or worse with the number of agents, the optimized implementation shows much closer to linear scaling.
2.  **Significant Gains at Scale:** The performance gap widens drastically as the agent count increases, reaching nearly a **200x improvement** at 500 agents.
3.  **Efficiency:** The optimization ("Faster Collision Checking Algorithm") effectively reduces the computational overhead of collision detection, making the system viable for much larger swarm sizes.

## Complexity Analysis
The optimized `mapf_post` implementation utilizes a sweep-line algorithm (sorting segments by their minimum X-coordinate) to prune collision checks.

### Exact Big-O Notation: **$O(NT \log(NT) + K)$**

Where:
*   **$N$**: Total number of agents.
*   **$T$**: Average trajectory length (number of waypoints).
*   **$S = NT$**: Total number of segments across all agents.
*   **$K$**: Number of pairs of segments whose X-axis intervals overlap.

### Breakdown:
1.  **Segment Generation:** $O(NT)$ – Computing AABBs for all $NT$ waypoints.
2.  **Sorting:** $O(S \log S) = O(NT \log(NT))$ – Sorting all segments by `min_x` for the sweep-line.
3.  **Collision Checking (Sweep-line):**
    *   **Average Case (Sparse):** $O(S + K) \approx O(NT)$ when agents are spatially distributed such that each segment only overlaps with a constant number of neighbors.
    *   **Worst Case (Dense):** $O(S^2) = O((NT)^2)$ if all segments' X-intervals overlap (e.g., all agents stacked vertically).

### Why the Benchmarks appear Linear:
The `agent_scaling` benchmark uses a spatial distribution where each agent is offset in X (`x = wp_idx - agent_idx`). This ensures that each agent only overlaps with a small, constant number of neighbors regardless of the total agent count $N$. Consequently, $K$ scales linearly with $NT$, making the observed performance appear $O(NT)$.

## Conclusion
The performance improvements in the `arjo/feat/more_efficient_checks` branch are substantial and critical for scaling the MAPF (Multi-Agent Path Finding) post-processing. Integration into `main` is highly recommended.
