# run in a terminal with: . setup.sh

# mount the S3 repository where some data are loaded from
mount-s3 --cache ~/.s3-cache/ gmto.im.grim ~/mnt
# path to CEO mirror modes files
export GMT_MODES_PATH=$HOME/mnt/ceo
# path to the FEM
export FEM_REPO=$HOME/mnt/20230530_1756_zen_30_M1_202110_FSM_202305_Mount_202305_noStairs
# mount model 
export MOUNT_MODEL=MOUNT_FDR_1kHz
# flowchart layout
export FLOWCHART=dot
# full path to CUDA compiler
export CUDACXX=/usr/local/cuda/bin/nvcc
# IP address to data server for scopes (only for AWS machine, comment out otherwise)
export SCOPE_SERVER_IP=`ec2metadata | sed -n 's/^local-ipv4: \(.*\)/\1/p'`
# location where the model write its data
export DATA_REPO=$HOME/projects/gmt-ns-im/web_server/static
