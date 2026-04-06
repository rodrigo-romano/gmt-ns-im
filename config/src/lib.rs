pub const SIM_SAMPLING_FREQUENCY: usize = 1000; // Hz
pub const BOOTSTRAPPING_DURATION: usize = 4; // seconds
pub const FAST_SEGMENT_TIPTILT_DURATION: usize = 1; // seconds
pub const HIGH_GAIN_ACO_DURATION: usize = 10; // seconds
pub const SIM_DURATION: usize =
    BOOTSTRAPPING_DURATION + FAST_SEGMENT_TIPTILT_DURATION + HIGH_GAIN_ACO_DURATION;

pub const ATMOSPHERE: bool = true;
// CFD 2025
pub const WINDLOADS: Option<&str> = Some("zen30az000_OS_7ms");
// CFD 2021
// pub const WINDLOADS: &str = Some("zen30az045_OS7");

pub mod m1 {
    // M1 polishing residual error figures (0: without, 1: with)
    pub const POLISH_ERROR_MAPS: usize = 1;
    pub mod segment {
        // use crate::m1::POLISH_ERROR_MAPS;

        pub const N_MODE: usize = 27;
        pub const N_RAW_MODE: usize = 335 + super::POLISH_ERROR_MAPS;
        pub const MODES: &str = concat!(env!("FEM_SHORT_ID"), "_m1_bending_modes");
        pub const RAW_MODES: &str = if super::POLISH_ERROR_MAPS == 0 {
            concat!(env!("FEM_SHORT_ID"), "_m1_raw_bending_modes")
        } else {
            concat!(env!("FEM_SHORT_ID"), "_m1_raw_bending_modes_polish12")
        };
        pub const ACTUATOR_RATE: usize = 10;
    }
    pub mod edge_sensor {
        pub const RBM_INTEGRATOR_GAIN: f64 = 0e-3;
    }
}

pub mod agws {
    pub mod sh24 {
        pub const RATE: usize = 5;
        // pub const INTEGRATOR_GAIN: f64 = 0.3;
        pub const POINTING_ERROR: Option<(f64, f64)> = None; // Some((150f64.from_mas(), -100f64.from_mas()));
        // use a resolved source
        pub const CALIBRATION_SRC: bool = !crate::ATMOSPHERE;
        // IIR filter coefficients (segment TT controller)
        pub mod simple_integrator {
            // Feed-forward coefficients
            pub const B_COEFFS: [f64; 1] = [-0.5];
            // Feedback coefficients (excluding a[0]=1.0)
            pub const A_COEFFS: [f64; 1] = [-1.0];
        }
        pub mod double_integrator_tustin {
            // Feed-forward coefficients
            pub const B_COEFFS: [f64; 3] = [-0.07128, -0.00495, 0.06633];
            // Feedback coefficients (excluding a[0]=1.0)
            pub const A_COEFFS: [f64; 2] = [-2.0, 1.0];
        }
        pub mod double_integrator_zoh {
            // Feed-forward coefficients
            pub const B_COEFFS: [f64; 2] = [-0.1426, 0.1327];
            // Feedback coefficients (excluding a[0]=1.0)
            pub const A_COEFFS: [f64; 2] = [-2.0, 1.0];
        }
        // Default double integrator
        pub use double_integrator_zoh as double_integrator;
    }
    pub mod sh48 {
        pub const RATE: usize = 1000;
        pub const INTEGRATOR_GAIN: f64 = 0.4;
        // use a resolved source
        pub const CALIBRATION_SRC: bool = !crate::ATMOSPHERE;
    }
}

pub mod fsm {
    pub const OFFLOAD_INTEGRATOR_GAIN: f64 = 0.; //1e-2;
}
