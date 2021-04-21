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
folder_name, mean radius, corrected sample standard deviation, fps
for example:
data/beton200/exp_200, r: 0.101436, std_dev: 0.000165287, fps: 3.86907
data/...

then all the radius' and MSE for each ply file:
data/beton200/exp_50
0.1023, 0.012
0.10123, 0.012
0.10321, 0.012
.
.
data/beton200/exp_100
0.102103o123, 0.012
0.1023123, 0.012
0.102315123, 0.012
.
.
.


full example:
../data/beton200/exp_100, r: 0.101641, std_dev: 0.000255073, fps: 3.62214
../data/beton200/exp_200, r: 0.101436, std_dev: 0.000165287, fps: 3.79161
../data/beton200/exp_300, r: 0.101427, std_dev: 0.000109219, fps: 3.75038
../data/beton200/exp_400, r: 0.101482, std_dev: 0.00013911, fps: 3.73944
../data/beton200/exp_50, r: 0.102607, std_dev: 0.000781884, fps: 3.86608
../data/beton200/exp_500, r: 0.101425, std_dev: 0.000133766, fps: 3.78301
radius, mse from: ../data/beton200/exp_100
0.10187, 1.27967e-05
0.101376, 1.28954e-05
0.101488, 1.29272e-05
0.101355, 1.29689e-05
0.10177, 1.37295e-05
0.101259, 1.29669e-05
0.101597, 1.28988e-05
0.10193, 1.31722e-05
0.101498, 1.29543e-05
0.101848, 1.35261e-05
0.101972, 1.39937e-05
0.10188, 1.33205e-05
0.101654, 1.30844e-05
0.101347, 1.27788e-05
0.101698, 1.28428e-05
0.101589, 1.25672e-05
0.102009, 1.29117e-05
0.101778, 1.33553e-05
0.102012, 1.32166e-05
0.101245, 1.28486e-05
0.101808, 1.32546e-05
0.101948, 1.29122e-05
0.101491, 1.28481e-05
0.102206, 1.30222e-05
0.101664, 1.25833e-05
0.101372, 1.29554e-05