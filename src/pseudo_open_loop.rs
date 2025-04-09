use std::sync::Arc;

use gmt_dos_clients_crseo::calibration::{Reconstructor, algebra::CalibProps};
use gmt_dos_clients_io::{gmt_m2::M2RigidBodyMotions, optics::SensorData};
use interface::{Data, Read, UID, Update, Write};

#[derive(UID)]
pub enum PseudoSensorData {}

#[derive(Debug, Default)]
pub struct PseudoOpenLoop {
    recon: Reconstructor,
    cmd: Arc<Vec<f64>>,
    slopes: Vec<f64>,
}

impl PseudoOpenLoop {
    pub fn new(recon: Reconstructor) -> Self {
        Self {
            recon,
            ..Default::default()
        }
    }
}

impl Update for PseudoOpenLoop {
    fn update(&mut self) {
        self.recon
            .calib()
            .zip(self.cmd.chunks(6))
            .for_each(|(calib, c)| {
                let cs = calib * &c[..calib.n_cols()];
                let mut iter = cs.col_as_slice(0).iter();
                self.slopes
                    .iter_mut()
                    .zip(calib.mask_as_slice())
                    .for_each(|(s, &m)| {
                        if m {
                            *s += *iter.next().unwrap()
                        }
                    })
            });
    }
}

impl Read<M2RigidBodyMotions> for PseudoOpenLoop {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        self.cmd = data.into_arc();
    }
}
impl Read<SensorData> for PseudoOpenLoop {
    fn read(&mut self, data: Data<SensorData>) {
        self.slopes = data.as_slice().to_vec();
    }
}
impl Write<PseudoSensorData> for PseudoOpenLoop {
    fn write(&mut self) -> Option<Data<PseudoSensorData>> {
        Some(self.slopes.clone().into())
    }
}
