mod merged_reconstructor;
pub mod calibration;
pub mod differential_reconstructor;
pub use merged_reconstructor::Sh48Reconstructor;

pub const TXY_RESIDUAL_SCALING: f64 = 150.0;

