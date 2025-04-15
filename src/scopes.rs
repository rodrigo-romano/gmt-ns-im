use gmt_dos_clients_io::{
    mount::AverageMountEncoders,
    optics::{SegmentPiston, SegmentTipTilt, SegmentWfeRms, TipTilt, WfeRms},
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_scope::scopehub;
use interface::{UID, units::Mas};

#[derive(UID)]
#[alias(port = 5001, name = SegmentPiston<-9>, client = LinearOpticalModel, traits = Write, Size)]
pub enum M1SegmentPiston {}
#[derive(UID)]
#[alias(port = 5002, name = SegmentPiston<-9>, client = LinearOpticalModel, traits = Write, Size)]
pub enum M2SegmentPiston {}
#[derive(UID)]
#[alias(port = 5003, name = Mas<SegmentTipTilt>, client = LinearOpticalModel, traits = Write, Size)]
pub enum M1SegmentTipTilt {}
#[derive(UID)]
#[alias(port = 5004, name = Mas<SegmentTipTilt>, client = LinearOpticalModel, traits = Write, Size)]
pub enum M2SegmentTipTilt {}

#[scopehub]
pub enum OnAxisScopes {
    Scope(WfeRms<-9>, SegmentWfeRms<-9>),
    Scope(SegmentPiston<-9>),
    Scope(Mas<TipTilt>, Mas<SegmentTipTilt>),
}
#[scopehub]
pub enum M1Scopes {
    Scope(M1SegmentPiston),
    Scope(M1SegmentTipTilt),
}
#[scopehub]
pub enum M2Scopes {
    Scope(M2SegmentPiston),
    Scope(M2SegmentTipTilt),
}
#[scopehub]
pub enum MountScopes {
    Scope(Mas<AverageMountEncoders>),
}
