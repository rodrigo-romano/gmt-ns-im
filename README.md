# GMT Natural Seeing Integrated Model

Integrated Model implementation of the GMT Natural Seeing Observatory Performance Mode.

## Setup

 1. Check the `setup.sh` script and update the path to reflect your own setup and run with `. setup.sh`
 2. Calibrations:
  * SH24 to FSM piezostack actuators displacement:
    1. follows the steps in `calibrations/README.md` to generate the FEM gain matrices
    2. then:
  ```shell
  cd sh24
  python pzt_2_rbm.py
  cargo r -r  
  ```
 3. Pre-compute the atmospheric turbulence model:
 ```shell
  cd atmosphere
  cargo r -r
```
 4. Copy the windloads data file `monitors.csv.z` from s3 (at the root) to the `gmt-ns-im` package folder
