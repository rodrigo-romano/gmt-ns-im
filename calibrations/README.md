The matrix T that transforms M2 Tz,Rx,Ry RBMs into FSM PZT actuators is derived from
$T K_1 = K_2$.

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

The matrix T is computed with the python script: `pzt_2_rbm.py`.

The poke matrix $D$ between AGWS SH24 and M2 Tz,Rx,Ry RBMs is computed in the
main script (`cargo r -r`).
It also computes the SH24 to PZT matrix $M = T D^{-1}$.
