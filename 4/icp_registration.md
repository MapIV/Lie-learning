# icp_registration

## How to run ICP
Provide two CSV files (target.csv and scan.csv) with two columns: x in the first column and y in the second column. Sample data are target.csv and source.csv.
```
python3 icp_registration.py <target.csv> <scan.csv> <output_folder>
```
## Operation example 
You can choose 2 optimization methods: Gauss-Newton or Levenberg-Marquardt.
Then, decide the initial pose while referring to the displayed graph. You can fix the initial pose.
```
Select optimization method [1: Gauss-Newton, 2: Levenberg-Marquardt] (default: 1) >> 1
<< Please set the initail pose >>
initial_x >> 7
initial_y >> 3.5
initial_theta >> 0
Are you sure you want to conduct ICP from this pose? No:0 Yes:1 >>1
```

Scan matching animation is saved in output_folder.