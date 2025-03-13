# SH24 to FSM piezostack actuactors calibration

$K_1$ is the FEM static gain between PZT forces and RBMS and
$K_2$ is the FEM static gain between PZT forces and displacements.

The FEM static gain matrix are computed with the `static_gain` binary from
the `gmt_dos-clients_fem` crate
```shell
cargo install --locked  gmt_dos-clients_fem --bin static_gain --features serde,clap
```
and the gains are obtained with:

 * K1: `static_gain -i MC_M2_PZT_F -o MC_M2_lcl_6D -f pzt_2_rbm.pkl`
 * K2: `static_gain -i MC_M2_PZT_F -o MC_M2_PZT_D -f pzt_f2d.pkl`

The matrix T that transforms M2 Rx,Ry RBMs into FSM PZT actuators is computed
with the python script: `pzt_2_rbm.py` following
the method described in GMT-DOC-05154.

The poke matrix $D$ between AGWS SH24 and M2 Rx,Ry RBMs is computed in the
main script (`cargo r -r`).
It also computes the SH24 to PZT matrix $M = T D^{-1}$.
