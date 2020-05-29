for f in *.stl
do
s=$f
ctmconv $f "${s/.stl/.ply}"
pcl_mesh_sampling "${s/.stl/.ply}" "${s/.stl/.pcd}" -leaf_size 0.001 -write_normals 0  
done
