Instructions for Benchmarking as of 4/30/2026


For now, there are 2 config files that need to match: benchmark.yaml and benchmark_ns.yaml. 
The only difference is namespacing. This will be resolved in the future so that there is only 1 config file.

Then run the safety corridor generating script: generate_sfc.yaml.
Then run the path generation script: local_traj_benchmark_mighty.yaml.

Now change the occlusion weight parameter (and any other params intended) in both config files, save and rebuild. 
Note that only weight should be changed, not global planner params.

Rerun the path generation script using a different name to save the output: local_traj_benchmark_mighty.yaml.

If you want to visualize both sets of trajectories, use trajectory_overlay.yaml and enable to modified_paths and nominal_paths MarkerArrays in Rviz.