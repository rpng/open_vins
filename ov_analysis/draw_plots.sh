
search_dir=./algorithms/ov_2.6.2_stereo
for yaml_dic in "$search_dir"/*
do
    echo "$yaml_dic"

    
    for seq_dic in "$yaml_dic"/*
	do
		echo "$seq_dic"
		
		for runs in "$seq_dic"/*
		do
		echo "$runs"
		evo_traj tum --ref=./groundtruth/${seq_dic##*/}.csv  "$runs" --save_plot ${runs%.txt}.png
		evo_ape tum ./groundtruth/${seq_dic##*/}.csv   "$runs"
		evo_rpe tum ./groundtruth/${seq_dic##*/}.csv  "$runs"
	
		
done
done
    
done
