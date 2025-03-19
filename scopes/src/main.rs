#![allow(unreachable_code)]

use gmt_dos_clients_io::{
    gmt_m2::fsm::M2FSMFsmCommand,
    optics::{SegmentPiston, SegmentTipTilt, SegmentWfeRms, TipTilt, WfeRms},
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_scope_client::Scope;
use interface::{UID, units::Mas};
use std::env;

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

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    loop {
        if let Ok(scope) = env::var("SCOPE") {
            match scope.as_str() {
                // "FSM" => Scope::new()
                //     .name("FSM Command")
                //     .signal::<M2FSMFsmCommand>()?
                //     .show(),
                // "M2 RBM" => Scope::new()
                //     .name("M2 RBM")
                //     .signal::<MuM<M2RigidBodyMotions>>()?
                //     .show(),
                "SegmentPiston" => Scope::new()
                    .name("Segment Piston")
                    .signal::<SegmentPiston<-9>>()?
                    .show(),
                "TipTilt" => Scope::new()
                    .name("Tip-Tilt")
                    .signal::<Mas<TipTilt>>()?
                    .signal::<Mas<SegmentTipTilt>>()?
                    .show(),
                "M1 M2 SegmentPiston" => Scope::new()
                    .name("M1 & M2 Segment Piston")
                    .signal::<M1SegmentPiston>()?
                    .signal::<M2SegmentPiston>()?
                    .show(),
                "M1 M2 SegmentTipTilt" => Scope::new()
                    .name("M1 & M2 Segment TipTilt")
                    .signal::<M1SegmentTipTilt>()?
                    .signal::<M2SegmentTipTilt>()?
                    .show(),
                // "M2" => Scope::new()
                //     .name("Segment Piston from M2 RBM")
                //     .signal::<M2SegmentPiston>()?
                //     .show(),
                // "M2RB" => Scope::new()
                //     .name("Segment Piston from M2 Reference Body RBM")
                //     .signal::<M2RBSegmentPiston>()?
                //     .show(),
                // "M2S" => Scope::new()
                //     .name("Segment Piston from M2 Shell Voice Coils")
                //     .signal::<M2SegmentMeanActuator>()?
                //     .show(),
                other => panic!("expected M1, M2, M2RB or M2S, found {other}"),
            }
        } else {
            Scope::new()
                .name("WFE RMS")
                .signal::<SegmentWfeRms<-9>>()?
                .signal::<WfeRms<-9>>()?
                .show();
        };
    }
    Ok(())
}
