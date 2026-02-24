use std::fs::File;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{integrator::Integrator, print::Print, timer::Timer};
use gmt_dos_clients_crseo::{
    OpticalModel,
    calibration::Reconstructor,
    crseo::{FromBuilder, Gmt},
    sensors::NoSensor,
};
use gmt_dos_clients_io::{gmt_m2::M2RigidBodyMotions, optics::WfeRms};
use gmt_dos_clients_optics_state::{M2State, MirrorState, OpticalState, OpticsState, SegmentState};
use gmt_dos_systems_agws::{
    Agws,
    agws::{
        sh24::{Sh24, Sh24TT, kernel::Sh24Kern},
        sh48::Sh48,
    },
    builder::shack_hartmann::ShackHartmannBuilder,
};
use interface::Tick;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("../calibrations/sh24/recon_sh24-to-rbm_pth.pkl")?,
        Default::default(),
    )?;
    let agws = Agws::<1, 1, Sh48<1>, Sh24TT<1>>::builder()
        .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
        .gmt(Gmt::builder().m1(
            config::m1::segment::RAW_MODES,
            config::m1::segment::N_RAW_MODE,
        ))
        .sh24_calibration(recon)
        .sh24_controller(Integrator::<M2RigidBodyMotions>::new(42).gain(0.5))
        .build()?;

    let m2 = MirrorState::default()
        .set_segment_state(1, SegmentState::rbms(&[1e-5, 0., 0., 0., 0., 0.]));
    let optical_state = OpticalState::default().zero_point(OpticalState::m2(m2));
    let m2_state = MirrorState::default();

    type AgwsSh24 = Sh24<1>;
    type AgwsSh24Kernel = Sh24Kern<Sh24TT<1>>;

    let print = Print::new(8);

    let timer: Timer = Timer::new(20);

    let on_axis = OpticalModel::<NoSensor>::builder().build()?;

    actorscript!(
        #[model(name=sh24)]
        1: timer[Tick] -> optical_state[OpticsState] -> {agws::AgwsSh24}
        1: {agws::AgwsSh24Kernel}[M2RigidBodyMotions]  
        1: {agws::AgwsSh24Kernel}[M2RigidBodyMotions]
            -> m2_state[M2State] -> optical_state
        1: optical_state[OpticsState] -> on_axis[WfeRms<-9>] -> print
    );

    Ok(())
}
