pub mod calibration;
pub mod differential_reconstructor;
mod kernels;
pub use kernels::{Sh48DiffReconstructor, Sh48Reconstructor};

pub const TXY_RESIDUAL_SCALING: f64 = 150.0;
