// mod merge_agws;
mod merged_reconstructor;
pub mod calibration;
use gmt_dos_clients_io::gmt_m1::{self, segment::ModeShapes};
use gmt_dos_clients_optics_state::MirrorState;
use interface::{Data, UniqueIdentifier, Write};
// pub use merge_agws::MergeAgws;
pub use merged_reconstructor::Sh48MergerReconstructor;

pub const TXY_RESIDUAL_SCALING: f64 = 150.0;

