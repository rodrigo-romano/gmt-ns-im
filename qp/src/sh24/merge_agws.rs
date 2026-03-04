use gmt_dos_clients_io::{Estimate, gmt_m2::M2RigidBodyMotions};
use gmt_dos_clients_optics_state::{M1State, M2State, MirrorState, SegmentState};
use interface::{Data, Read, Update, Write};

use super::{M1_N_MODE, TXY_RESIDUAL_SCALING};

pub struct MergeAgws {
    m2_rbms: Vec<f64>,
    m1_modes: Vec<f64>,
}
impl MergeAgws {
    pub fn new() -> Self {
        Self {
            m2_rbms: vec![0f64; 42],
            m1_modes: vec![0f64; M1_N_MODE * 7],
        }
    }
}
impl Update for MergeAgws {}
impl Read<M2RigidBodyMotions> for MergeAgws {
    fn read(&mut self, data: Data<M2RigidBodyMotions>) {
        // dbg!(&data);
        self.m2_rbms
            .chunks_mut(6)
            .zip(data.chunks(6))
            .for_each(|(rbms, data)| {
                rbms[3] = data[3];
                rbms[4] = data[4];
            });
    }
}
impl Read<Estimate> for MergeAgws {
    fn read(&mut self, data: Data<Estimate>) {
        // dbg!(&data);
        self.m2_rbms
            .chunks_mut(6)
            .zip(data.chunks(6 + M1_N_MODE))
            .for_each(|(rbms, data)| {
                rbms[0] = data[0] * TXY_RESIDUAL_SCALING;
                rbms[1] = data[1] * TXY_RESIDUAL_SCALING;
            });
        self.m1_modes
            .chunks_mut(M1_N_MODE)
            .zip(data.chunks(6 + M1_N_MODE))
            .for_each(|(modes, data)| {
                modes.clone_from_slice(&data[6..]);
            });
    }
}
impl Write<M2State> for MergeAgws {
    fn write(&mut self) -> Option<Data<M2State>> {
        Some(Data::new(MirrorState::from_rbms(&self.m2_rbms).into()))
    }
}
impl Write<M1State> for MergeAgws {
    fn write(&mut self) -> Option<Data<M1State>> {
        let m1: MirrorState = self
            .m1_modes
            .chunks(M1_N_MODE)
            .map(|modes| SegmentState::modes(modes))
            .collect();
        Some(Data::new(m1))
    }
}
