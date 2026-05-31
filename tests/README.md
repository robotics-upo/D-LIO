# viral_eval
Matlab scripts to calculate the position estimation error, this is part of the NTU VIRAL DATA SUITE

Based on the original [viral_eval](https://github.com/ntu-aris/viral_eval) tools.
Adapted for D-LIO odometry output (`dlio.csv` from `nav_msgs/Odometry`).

Please find the instruction for usage at our [web page](https://ntu-aris.github.io/ntu_viral_dataset/evaluation_tutorial.html)

## Directory Structure

```
tests/
├── evaluate_all.m          # Main entry point: evaluates all sequences
├── evaluate_one.m          # Single-sequence ATE evaluator (adapted for D-LIO)
├── combteeth.m             # Time-stamp association utility
├── quatconv.m              # Quaternion rotation utility
├── traj_align.m            # Optimal trajectory alignment (SVD)
├── vecitp.m                # Vector interpolation utility
├── tightfig.m              # Figure layout utility
├── trans_B2prism.csv       # Body-to-prism offset for VIRAL platform
└── Dataset/
    └── VIRAL/
        └── result_*/       # Each contains: leica_pose.csv + dlio.csv
```

## Usage

1. Place your D-LIO results as `dlio.csv` in `Dataset/VIRAL/result_<sequence>/`
2. Ensure `leica_pose.csv` (ground truth) is present in the same folder
3. Open `evaluate_all.m` in MATLAB and run (F5)

## Citation

If you use this resource, please cite our work

```
@article{nguyen2021ntuviral,
  title={NTU VIRAL: A Visual-Inertial-Ranging-Lidar dataset, from an aerial vehicle viewpoint},
  author={Nguyen, Thien-Minh and Yuan, Shenghai and Cao, Muqing and Lyu, Yang and Nguyen, Thien Hoang and Xie, Lihua},
  journal={Submitted to IJRR},
  year={2021}
}
```
