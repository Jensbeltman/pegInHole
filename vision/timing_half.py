import numpy as np
rating = np.array([2,5,5,4,2,1,1,5,5,5])

scene_kps = np.array([856,686,749,802,833,761,645,949,847,865])
cluster_kps = np.array([802])
time_corr_gen = np.array([0.336757,0.250723,0.278575,0.302765,0.304372,0.268125,0.241538,0.339995,0.320816,0.33205])
time_sac_gen = np.array([0.089396,0.06868,0.076074,0.078008,0.080603,0.074453,0.066772,0.090683,0.082954,0.084404])
time_icp_gen = np.array([0.037648,0.016321,0.017219,0.017506,0.032596,0.017317,0.055732,0.018023,0.02034,0.035211])
time_total_gen = np.array([0.463801,0.335724,0.371868,0.398279,0.417571,0.359895,0.364042,0.448701,0.42411,0.451665])

print(np.average(rating),np.std(rating))
print(np.average(scene_kps),np.std(scene_kps))
print(np.average(cluster_kps),np.std(cluster_kps))
print(np.average(time_corr_gen),np.std(time_corr_gen))
print(np.average(time_sac_gen),np.std(time_sac_gen))
print(np.average(time_icp_gen),np.std(time_icp_gen))
print(np.average(time_total_gen),np.std(time_total_gen))

# Found 856 between cluster 0 and model 0 of 856/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.336757
# SAC: 0.089396
# ICP: 0.037648
# Total: 0.463801 s

# Found 686 between cluster 0 and model 0 of 686/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.250723
# SAC: 0.06868
# ICP: 0.016321
# Total: 0.335724 s

# Found 749 between cluster 0 and model 0 of 749/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.278575
# SAC: 0.076074
# ICP: 0.017219
# Total: 0.371868 s

# Found 802 between cluster 0 and model 0 of 802/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.302765
# SAC: 0.078008
# ICP: 0.017506
# Total: 0.398279 s

# Found 833 between cluster 0 and model 0 of 833/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.304372
# SAC: 0.080603
# ICP: 0.032596
# Total: 0.417571 s

# Found 761 between cluster 0 and model 0 of 761/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.268125
# SAC: 0.074453
# ICP: 0.017317
# Total: 0.359895 s

# Found 645 between cluster 0 and model 0 of 645/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.241538
# SAC: 0.066772
# ICP: 0.055732
# Total: 0.364042 s

# Found 949 between cluster 0 and model 0 of 949/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.339995
# SAC: 0.090683
# ICP: 0.018023
# Total: 0.448701 s

# Found 847 between cluster 0 and model 0 of 847/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.320816
# SAC: 0.082954
# ICP: 0.02034
# Total: 0.42411 s

# Found 865 between cluster 0 and model 0 of 865/802
# ICP converged.
# CPU time used: 
# CorrMatch: 0.33205
# SAC: 0.084404
# ICP: 0.035211
# Total: 0.451665 s
