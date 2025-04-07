# M1 Bending Modes

The bending modes are derived from the telescope FEM and built such that the forces applied to the actuators to shape a bending mode onto an M1 segment do not cause any rigid body motion (RBM) of the segment.

Lets  define the actuator air cylinder force vector as $f$.
The RBM of an M1 segment $r$ are given by $r = G_r f$ where $G_r$ is the FEM static gain from M1 actuator forces to M1 RBMs.
The surface axial displacements $\delta$ of the same M1 segment are given by $\delta = G_\delta f$ where $G_\delta$ is the static gain from M1 actuator forces to M1 axial displacements.

Given the SVD of both gain matrices:
* $G_r=U_r S_r V_r^T$ , $V_r$ is $[335,6]$
* $G_\delta=U_\delta S_\delta V_\delta^T$
The right eigen vectors of $G_r$ are removed from the right eigen vectors of $G_\delta$:
$V_{\delta r} = V_\delta - V_r q$ where $q = V_r^T V_\delta$.

Recomputing $G_\delta$ as $\bar G_\delta=\bar U_\delta \bar S_\delta \bar V_{\delta r}^T$ and evaluating the SVD of $\bar G_\delta$:
$\bar G_\delta = U S V^T = B M$ where
 * $B=U$ are the bending modes and
 * $M=S V$ is force to mode matrix.
 
 Lets derived the RBM corresponding to the surface $\Sigma = B b$, the actuator force vector is $f=M^{-1} B^T \Sigma$ and the RBM are $r = G_r M^{-1} B^T \Sigma$.
 After substitution, the RBM are written:
 $r = U_r S_r V_r^T V S^{-1} B^T \Sigma = U_r S_r V_r^T V_{\delta r} S_\delta^{-1} U_\delta^T \Sigma = 0$
 as $V_r^T V_{\delta r} = V_r^T (V_\delta - V_r q) = q -q = 0$ owing to the property $V_r^TV_r=I$.
 
The bending modes are computed by running the binary `m1-modes` that is installed with:
```shell
cargo install --locked gmt_dos-systems_m1-modes --git https://github.com/rconan/dos-actors.git --branch gmt-ns-im
```

Then the `CEO` mode shapes and the mode to force transformation are generated and saved to files with:
```shell
  python m1_fem_bending_modes.py
```
