# run in a terminal with: . setup.sh

# mount the S3 repository where some data are loaded from
if mountpoint -q ~/mnt; then
    echo "gmto.im.grim is already mounted at ~/mnt, skipping."
else
    mount-s3 --cache ~/.s3-cache/ gmto.im.grim ~/mnt
fi

# path to CEO mirror modes files
export GMT_MODES_PATH=$HOME/Workspace/gmt-data/ceo_data
#$HOME/mnt/ceo
# path to the FEM
export FEM_REPO=$HOME/mnt/20250506_1715_zen_30_M1_202110_FSM_202305_Mount_202305_pier_202411_M1_actDamping
# mount model 
export MOUNT_MODEL=MOUNT_FDR_1kHz
# flowchart layout (neato or dot)
export FLOWCHART=neato
# full path to CUDA compiler
export CUDACXX=/usr/local/cuda-12.1/bin/nvcc
# IP address to data server for scopes (only for AWS machine, comment out otherwise)
# export SCOPE_SERVER_IP=`ec2metadata | sed -n 's/^local-ipv4: \(.*\)/\1/p'`
# location where the model write its data
export DATA_REPO=$HOME/Workspace/gr-ns-im/web_server/static
