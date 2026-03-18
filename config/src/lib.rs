pub const SIM_SAMPLING_FREQUENCY: usize = 1000; // Hz
pub const SIM_DURATION: usize = 60_usize; // seconds
pub const BOOTSTRAPPING_DURATION: usize = 4_usize; // seconds

pub const ATMOSPHERE: bool = true;
// pub const WINDLOADS: bool = true;

pub mod m1 {
    // M1 polishing residual error figures (0: without, 1: with)
    pub const POLISH_ERROR_MAPS: usize = 1;
    pub mod segment {
        // use crate::m1::POLISH_ERROR_MAPS;

        pub const N_MODE: usize = 27;
        pub const N_RAW_MODE: usize = 335 + super::POLISH_ERROR_MAPS;
        pub const MODES: &str = "20230530_1756_m1_bending_modes";
        pub const RAW_MODES: &str = if super::POLISH_ERROR_MAPS == 0 {
            "20230530_1756_m1_raw_bending_modes"
        } else {
            "20230530_1756_m1_raw_bending_modes_polish12"
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
        pub const INTEGRATOR_GAIN: f64 = 0.3;
        pub const POINTING_ERROR: Option<(f64, f64)> = None; // Some((150f64.from_mas(), -100f64.from_mas()));
        // use a resolved source
        pub const CALIBRATION_SRC: bool = !crate::ATMOSPHERE;
    }
    pub mod sh48 {
        pub const RATE: usize = 1000;
        pub const INTEGRATOR_GAIN: f64 = 0.4;
        // use a resolved source
        pub const CALIBRATION_SRC: bool = !crate::ATMOSPHERE;
    }
}

pub mod fsm {
    pub const OFFLOAD_INTEGRATOR_GAIN: f64 = 1e-2;
}
