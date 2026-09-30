# Development roadmap

1. Validate forklift motion in Gazebo Harmonic.
2. Connect the allocator to real AMR/forklift state topics.
3. Tune the SDF-derived A* inflation radius to the actual vehicle footprint.
4. Add dynamic obstacle layers from `/scan`.
5. Add local conflict prediction and direct robot-to-robot negotiation.
6. Add Zenoh transport and the Normal → Degraded → Emergency communication modes.
7. Add battery-aware and congestion-aware replanning.
8. Add D* Lite or another incremental replanner for dynamic changes.
