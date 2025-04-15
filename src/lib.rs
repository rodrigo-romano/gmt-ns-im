#[derive(interface::UID)]
pub enum M2RBMasSH48 {}
#[derive(interface::UID)]
pub enum MountEstimate {}

#[cfg(feature = "scope")]
pub mod scopes;

pub mod m1_bending_modes;
mod pseudo_open_loop;
pub use pseudo_open_loop::{PseudoOpenLoop, PseudoSensorData};

pub mod config {
    pub const ATMOSPHERE: bool = false;
    pub mod m1 {
        pub mod segment {
            pub const N_MODE: usize = 27;
            pub const N_RAW_MODE: usize = 335;
            pub const MODES: &str = "20230530_1756_m1_bending_modes";
            pub const RAW_MODES: &str = "20230530_1756_m1_raw_bending_modes";
            pub const ACTUATOR_RATE: usize = 10;
        }
        pub mod edge_sensor {
            pub const RBM_INTEGRATOR_GAIN: f64 = 0.; //1e-3;
        }
    }
    pub mod agws {
        pub mod sh24 {
            pub const RATE: usize = 5;
            pub const INTEGRATOR_GAIN: f64 = 0.2;
            pub const POINTING_ERROR: Option<(f64, f64)> = None;
            // Some((150f64.from_mas(), -100f64.from_mas()));
        }
        pub mod sh48 {
            pub const RATE: usize = 50;
        }
    }
    pub mod fsm {
        pub const OFFLOAD_INTEGRATOR_GAIN: f64 = 1e-3;
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
