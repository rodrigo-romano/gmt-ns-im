use std::fs::File;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    gain::Gain, operator::Operator, print::Print, select::Select, signals::Signals,
};
use gmt_dos_clients_io::{
    gmt_m1::segment::ModeShapes,
    optics::{
        M1State,
        state::{MirrorState, OpticalState, SegmentState},
    },
};
use gmt_dos_clients_servos::{GmtFem, GmtM1, GmtServoMechanisms, M1SegmentFigure};
use gmt_dos_systems_m1::SingularModes;
use gmt_fem::FEM;
use gmt_ns_im::config;
use interface::{Left, Right, optics::OpticsState};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let sim_sampling_frequency = 1000;
    let n_sim = 10_000;

    let fem = FEM::from_env()?;

    let m1_sms: SingularModes = serde_pickle::from_reader(
        &File::open("calibrations/m1/modes/m1_singular_modes.pkl")?,
        Default::default(),
    )?;

    let b2f = m1_sms.mode2force();
    let s2bb: Vec<_> = m1_sms
        .modes_into_mat()
        .into_iter()
        .map(|x| x.columns(0, 5).transpose())
        .collect();

    let servos = GmtServoMechanisms::<{ config::m1::segment::ACTUATOR_RATE }, 1>::new(
        sim_sampling_frequency as f64,
        fem,
    )
    .m1_segment_figure(M1SegmentFigure::new().transforms(s2bb).modes_to_forces(b2f))
    .build()?;

    let optical_state = OpticalState::m1(
        MirrorState::default()
            .set_segment_state(1, SegmentState::modes(vec![0f64; 329]).set_mode(3, -2.5e-6)),
    );

    let print = Print::new(6);
    let sel = Select::new(0..5);

    let m1_bm = Signals::new(329, n_sim).channel(0, 1e-6);

    let gg = Gain::new(vec![1e6; 5]);

    let add_m1_state = Operator::<_>::plus();

    let print_state = Print::new(8).tag("State");

    actorscript!(
        #[model(name=m1_modal_ctrl)]
        1: m1_bm[ModeShapes<1>] -> {servos::GmtM1}
        1: {servos::GmtFem}[ModeShapes<1>] -> sel
        1: optical_state[Right<OpticsState>] ->add_m1_state
        1: {servos::GmtFem}[Left<OpticsState>] -> add_m1_state
        1000: add_m1_state[OpticsState] -> print_state
        1000: sel[ModeShapes<1>] -> gg[ModeShapes<1>] -> print
    );
    Ok(())
}
