#[derive(interface::UID)]
pub enum M2RBMasSH48 {}
#[derive(interface::UID)]
pub enum MountEstimate {}

#[cfg(feature = "scope")]
pub mod scopes;

pub mod m1_bending_modes;
mod merge;
mod pseudo_open_loop;
pub use merge::{MergeReconstructor, SplitEstimate};
pub use pseudo_open_loop::{PseudoOpenLoop, PseudoSensorData};

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
