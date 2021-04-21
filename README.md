# ROB6


folder structure:
data:
	beton200:
		exp_50
		exp_100
		.
		.
		.
		result_ransac.txt
		result_ipc.txt
	dry_pvc_400
	.
	.
	.
	

each result file contains:
folder_name, mean radius, corrected sample standard deviation, MSE, fps
for example:
data/beton200/exp_200, r: 0.101436, std_dev: 0.000165287, fps: 3.86907
data/...

then all the radius' for each ply file:
data/beton200/exp_50
0.1023
0.10123
0.10321
.
.
data/beton200/exp_100
0.102103o123
0.1023123
0.102315123
.
.
.


full example:
../data/beton200/exp_100, r: 0.101641, std_dev: 0.000255073, fps: 3.80518
../data/beton200/exp_200, r: 0.101436, std_dev: 0.000165287, fps: 3.86907
../data/beton200/exp_300, r: 0.101427, std_dev: 0.000109219, fps: 3.84349
../data/beton200/exp_400, r: 0.101482, std_dev: 0.00013911, fps: 3.85475
../data/beton200/exp_50, r: 0.102607, std_dev: 0.000781884, fps: 3.92403
../data/beton200/exp_500, r: 0.101425, std_dev: 0.000133766, fps: 3.85416
points from: ../data/beton200/exp_100
0.101334
0.10193
0.101472
0.101259
0.101242
0.101848
0.102091
0.101914
0.101664
0.101737
0.101355
0.101948
0.101331
0.102012
0.101376
0.101785
0.101771
0.101808
0.101607
0.101744
0.101488
0.101518
0.101575
0.102206
0.101245
0.101972
0.101454
0.10187
0.101345
0.10177
