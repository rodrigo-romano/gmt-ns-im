## M1 EDGE SENSORS TO M1 RBM

$K_1$ is the FEM static gain between M1 hardpoints forces and RBMS and
$K_2$ is the FEM static gain between M1 hardpoints forces and edge sensors.

The FEM static gain matrix are computed with the `static_gain` binary from
the `gmt_dos-clients_fem` crate
```shell
cargo install --locked  gmt_dos-clients_fem --bin static_gain --features serde,clap
```
and the gains are obtained with:

 * K1: `static_gain -i OSS_Harpoint_delta_F -o OSS_M1_lcl -f hardpoints_2_rbm.pkl`
 * K2: `static_gain -i OSS_Harpoint_delta_F -o OSS_M1_edge_sensors -f hardpoints_2_edge-sensors.pkl`

The matrix T that transforms M1 edge sensors into M1 RBMs is computed
with the python script: `es_2_rbm.py` where T is solutions of $K_1 = T K_2$
