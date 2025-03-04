# run in a terminal with: . setup.sh
mount-s3 --cache ~/.s3-cache/ gmto.im.grim ~/mnt
export GMT_MODES_PATH=$HOME/mnt/ceo
export FEM_REPO=$HOME/mnt/20230530_1756_zen_30_M1_202110_FSM_202305_Mount_202305_noStairs
export MOUNT_MODEL=MOUNT_FDR_1kHz
export FLOWCHART=dot
export CUDACXX=/usr/local/cuda/bin/nvcc
# export SCOPE_SERVER_IP=`ec2metadata | sed -n 's/^local-ipv4: \(.*\)/\1/p'`
