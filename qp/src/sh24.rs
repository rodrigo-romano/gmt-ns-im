mod merge_agws;
mod merged_reconstructor;
pub mod calibration;
use gmt_dos_clients_io::gmt_m1::{self, segment::ModeShapes};
use gmt_dos_clients_optics_state::MirrorState;
use interface::{Data, UniqueIdentifier, Write};
pub use merge_agws::MergeAgws;
pub use merged_reconstructor::Sh48MergerReconstructor;

pub const M1_N_MODE: usize = 9;
pub const TXY_RESIDUAL_SCALING: f64 = 150.0;

pub enum M1RBM<const ID: u8> {}
impl<const ID: u8> UniqueIdentifier for M1RBM<ID> {
    type DataType = <gmt_m1::segment::RBM<ID> as UniqueIdentifier>::DataType;
    const PORT: u16 = 51_110 + ID as u16;
}
impl<const ID: u8> Write<M1RBM<ID>> for MirrorState {
    fn write(&mut self) -> Option<Data<M1RBM<ID>>> {
        <_ as Write<gmt_m1::segment::RBM<ID>>>::write(self).map(|data| data.transmute())
    }
}

#[gmt_dos_clients_scope::scopehub]
pub enum M1RBMScope {
    Scope(M1RBM<1>),
    Scope(M1RBM<2>),
    Scope(M1RBM<3>),
    Scope(M1RBM<4>),
    Scope(M1RBM<5>),
    Scope(M1RBM<6>),
    Scope(M1RBM<7>),
    Scope(ModeShapes<1>),
    Scope(ModeShapes<2>),
    Scope(ModeShapes<3>),
    Scope(ModeShapes<4>),
    Scope(ModeShapes<5>),
    Scope(ModeShapes<6>),
    Scope(ModeShapes<7>),
}

pub enum M2RBM<const ID: u8> {}
impl<const ID: u8> UniqueIdentifier for M2RBM<ID> {
    type DataType = <gmt_m1::segment::RBM<ID> as UniqueIdentifier>::DataType;
    const PORT: u16 = 52_220 + ID as u16;
}
impl<const ID: u8> Write<M2RBM<ID>> for MirrorState {
    fn write(&mut self) -> Option<Data<M2RBM<ID>>> {
        <_ as Write<gmt_m1::segment::RBM<ID>>>::write(self).map(|data| data.transmute())
    }
}

#[gmt_dos_clients_scope::scopehub]
pub enum M2RBMScope {
    Scope(M2RBM<1>),
    Scope(M2RBM<2>),
    Scope(M2RBM<3>),
    Scope(M2RBM<4>),
    Scope(M2RBM<5>),
    Scope(M2RBM<6>),
    Scope(M2RBM<7>),
}
