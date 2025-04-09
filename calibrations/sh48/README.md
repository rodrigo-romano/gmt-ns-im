# M1 & M2 RBM and M1 modes calibration with AGWS SH48

There is 2 variants for the calibrations: open-loop and closed-loop, closed-loop assumes that M2 Rx and Ry RBMs are corrected using the AGWS SH44.

The different calibrations are selected using `cargo` [features](https://doc.rust-lang.org/cargo/reference/features.html). 

The table below relates calibrated modes and features:

| Mode | Feature |
|------|---------|
| M1 RBM | m1_rbm |
| M2 RBM | m2_rbm |
| M1 BM | m1_bm |

Open or closed loop calibration is selected with the features `open_loop` or `closed_loop`, respectively.

For example, the calibration in open-loop of M1 RBM is obtained with
```shell
cargo r -r --features open_loop,m1_rbm  
```
and in closed-loop with
```shell
cargo r -r --features closed_loop,m1_rbm  
```

All the calibrations in either open or closed loop are performed with
```shell
cargo r -r --features open_loop
```
and in closed-loop with
```shell
cargo r -r --features closed_loop
```

Finally, all the calibrations for both open and closed are realized with
```shell
cargo r -r --features all
```

