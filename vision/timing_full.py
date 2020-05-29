import numpy as np
rating = np.array([1,4,2,4,1,1,1,3,1,1])
scene_kps = np.array([856,686,749,802,833,761,645,949,847,865])
cluster_kps = np.array([1598])
time_corr_gen = np.array([0.679445,0.500388,0.549433,0.596715,0.601216,0.561639,0.472949,0.695859,0.623197,0.647598])
time_sac_gen = np.array([0.089514,0.069273,0.074436,0.078494,0.080833,0.076844,0.064728,0.091581,0.081507,0.084886])
time_icp_gen = np.array([0.086376,0.112939,0.081158,0.054216,0.092335,0.061194,0.076342,0.071854,0.107748,0.057012])
time_total_gen = np.array([0.855335,0.6826,0.705027,0.729425,0.774381,0.699677,0.614019,0.859294,0.812452,0.789496])

print(np.average(rating),np.std(rating))
print(np.average(scene_kps),np.std(scene_kps))
print(np.average(cluster_kps),np.std(cluster_kps))
print(np.average(time_corr_gen),np.std(time_corr_gen))
print(np.average(time_sac_gen),np.std(time_sac_gen))
print(np.average(time_icp_gen),np.std(time_icp_gen))
print(np.average(time_total_gen),np.std(time_total_gen))
# Found 856 between cluster 0 and model 1 of 856/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.679445
# SAC: 0.089514
# ICP: 0.086376
# Total: 0.855335 s

# Found 686 between cluster 0 and model 1 of 686/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.500388
# SAC: 0.069273
# ICP: 0.112939
# Total: 0.6826 s

# Found 749 between cluster 0 and model 1 of 749/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.549433
# SAC: 0.074436
# ICP: 0.081158
# Total: 0.705027 s

# Found 802 between cluster 0 and model 1 of 802/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.596715
# SAC: 0.078494
# ICP: 0.054216
# Total: 0.729425 s

# Found 833 between cluster 0 and model 1 of 833/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.601216
# SAC: 0.08083
# ICP: 0.092335
# Total: 0.774381 s

# Found 761 between cluster 0 and model 1 of 761/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.561639
# SAC: 0.076844
# ICP: 0.061194
# Total: 0.699677 s

# Found 645 between cluster 0 and model 1 of 645/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.472949
# SAC: 0.064728
# ICP: 0.076342
# Total: 0.614019 s

# Found 949 between cluster 0 and model 1 of 949/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.695859
# SAC: 0.091581
# ICP: 0.071854
# Total: 0.859294 s

# Found 847 between cluster 0 and model 1 of 847/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.623197
# SAC: 0.081507
# ICP: 0.107748
# Total: 0.812452 s

# Found 865 between cluster 0 and model 1 of 865/1598
# ICP converged.
# CPU time used: 
# CorrMatch: 0.647598
# SAC: 0.084886
# ICP: 0.057012
# Total: 0.789496 s