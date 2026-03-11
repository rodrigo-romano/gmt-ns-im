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
        pub const RBM_INTEGRATOR_GAIN: f64 = 0e-3;
    }
}
pub mod agws {
    pub mod sh24 {
        pub const RATE: usize = 5;
        pub const INTEGRATOR_GAIN: f64 = 0.1;
        pub const POINTING_ERROR: Option<(f64, f64)> = None;
        // Some((150f64.from_mas(), -100f64.from_mas()));
    }
    pub mod sh48 {
        pub const RATE: usize = 5000;
    }
}
pub mod fsm {
    pub const OFFLOAD_INTEGRATOR_GAIN: f64 = 1e-2;
}
