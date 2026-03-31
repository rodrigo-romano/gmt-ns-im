#[derive(interface::UID)]
pub enum M2RBMasSH48 {}
#[derive(interface::UID)]
pub enum MountEstimate {}

// #[cfg(feature = "scope")]
// pub mod scopes;

pub mod m1_bending_modes;
mod merge;
mod pseudo_open_loop;

use std::sync::Arc;

use faer::{Mat, MatRef};
use gmt_dos_clients::integrator::Offset;
use gmt_dos_clients_io::{Estimate, gmt_m2::fsm::M2FSMFsmCommand};
use interface::{Data, Left, Read, UniqueIdentifier, Update, Write};
use matio_rs::MatFile;
pub use merge::{MergeReconstructor, SplitEstimate};
pub use pseudo_open_loop::{PseudoOpenLoop, PseudoSensorData};
pub mod agws;

#[derive(interface::UID)]
pub enum M2Txy {}

pub struct M2TxyToRxy {
    rxy_2_pzt: Vec<Mat<f64>>,
    rxy: Option<Vec<Vec<f64>>>,
    pzt: Option<Vec<f64>>,
    data: Arc<Vec<f64>>,
}

impl M2TxyToRxy {
    const TXY_TO_RXY: f64 = 0.24;
    pub fn new() -> anyhow::Result<Self> {
        let matfile = MatFile::load("calibrations/sh24/rbm_2_pzt_pth.mat")?;
        let mut mat = vec![];
        for i in 0..7 {
            let var: Vec<f64> = matfile.var(format!("var{i}"))?;
            let mat_ref = MatRef::from_column_major_slice(&var, 3, 2);
            println!("Rxy to PZT #{}: {:?}", i + 1, mat_ref.shape());
            mat.push(mat_ref.to_owned());
        }
        Ok(Self {
            rxy_2_pzt: mat,
            rxy: None,
            pzt: None,
            data: Default::default(),
        })
    }
}

impl Update for M2TxyToRxy {
    fn update(&mut self) {
        // if let Some(rxy) = self.rxy.take() {
        //     self.pzt = Some(
        //         self.rxy_2_pzt
        //             .iter()
        //             .zip(rxy.into_iter())
        //             .map(|(mat, txy)| mat * MatRef::from_column_major_slice(txy.as_slice(), 2, 1))
        //             .flat_map(|pzt| {
        //                 pzt.col_iter()
        //                     .flat_map(|c| c.iter().copied())
        //                     .collect::<Vec<f64>>()
        //             })
        //             .collect(),
        //     )
        // } else {
        //     self.pzt = None;
        // }
    }
}
impl<U: UniqueIdentifier<DataType = Vec<f64>>> Read<U> for M2TxyToRxy {
    fn read(&mut self, data: Data<U>) {
        self.data = data.as_arc();
        self.rxy = Some(
            data.chunks(6)
                .map(|rbms| vec![rbms[1] * Self::TXY_TO_RXY, rbms[0] * -Self::TXY_TO_RXY])
                .collect(),
        );
    }
}
impl Write<Offset<M2FSMFsmCommand>> for M2TxyToRxy {
    fn write(&mut self) -> Option<Data<Offset<M2FSMFsmCommand>>> {
        Some(Data::new(self.pzt.take()))
    }
}

impl Write<Left<Estimate>> for M2TxyToRxy {
    fn write(&mut self) -> Option<Data<Left<Estimate>>> {
         self
            .rxy
            .take()
            .map(|rxy| {
                rxy.into_iter()
                    .zip(self.data.chunks(6))
                    .flat_map(|(rxy, rbms)| {
                        let mut rbms_p = rbms.to_vec();
                        // &mut rbms_p[3..5].copy_from_slice(rxy.as_slice());
                        rbms_p[3] = rxy[0];
                        rbms_p[4] = rxy[1];
                        rbms_p
                    }).collect::<Vec<_>>()
            }).map(|x| x.into())
        
    }
}

// static agws: Sys<Agws<{ config::agws::sh48::RATE }, { config::agws::sh24::RATE }>> = {
//     let recon: Reconstructor = serde_pickle::from_reader(
//         File::open("calibrations/sh24/recon_sh24-to-pzt_pth.pkl")?,
//         Default::default(),
//     )?;

//     if config::ATMOSPHERE {
//         Agws::builder()
//             .load_atmosphere("atmosphere/atmosphere.toml", sim_sampling_frequency as f64)?
//     } else {
//         Agws::builder().sh24(ShackHartmannBuilder::sh24().use_calibration_src())
//     }
//     .gmt(Gmt::builder().m1(
//         gmt_ns_im::config::m1::segment::RAW_MODES,
//         gmt_ns_im::config::m1::segment::N_RAW_MODE,
//     ))
//     .sh24_calibration(recon)
//     .build()?
// };
