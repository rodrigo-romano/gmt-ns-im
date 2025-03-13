use std::{fs::File, time::Instant};

use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{gif, integrator::Integrator, timer::Timer};
use gmt_dos_clients_crseo::{
    OpticalModel, calibration::Reconstructor, crseo::builders::AtmosphereBuilder, sensors::NoSensor,
};
// use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::Exponential};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::{M2RigidBodyMotions, fsm::M2FSMFsmCommand},
    optics::{Frame, Host, SegmentPiston, SegmentWfeRms, Wavefront, WfeRms},
};
use gmt_dos_clients_scope::server::{Monitor, Scope};
use gmt_dos_clients_servos::{GmtFem, GmtM2, GmtServoMechanisms};
use gmt_dos_systems_agws::{
    Agws,
    agws::{sh24::Sh24, sh48::Sh48},
    kernels::Kernel,
};
use gmt_fem::FEM;
use interface::Tick;

const ACTUATOR_RATE: usize = 10;
const SH48_RATE: usize = 5000;
const SH24_RATE: usize = 100;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    println!("FEM  : {}", env!("FEM_REPO"));
    println!("MOUNT: {}", env!("MOUNT_MODEL"));

    let now = Instant::now();

    let sim_sampling_frequency = 1000;
    let sim_duration = 10_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let fem = FEM::from_env()?;
    // println!("{fem}");

    let servos =
        GmtServoMechanisms::<ACTUATOR_RATE, 1>::new(sim_sampling_frequency as f64, fem).build()?;

    // AGWS
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/recon_sh24-to-pzt_pth.pkl")?,
        Default::default(),
    )?;
    let agws: Sys<Agws<SH48_RATE, SH24_RATE>> = Agws::<SH48_RATE, SH24_RATE>::builder()
        .load_atmosphere("atmosphere/atmosphere.toml", sim_sampling_frequency as f64)?
        .sh24_calibration(recon)
        .build()?;
    println!("{agws}");

    let sh48_frame: gif::Frame<f32> = gif::Frame::new("sh48_frame.png", 48 * 8);
    let sh24_frame: gif::Frame<f32> = gif::Frame::new("sh24_frame.png", 24 * 12);

    // FSM command integrator
    let fsm_pzt_int = Integrator::new(21).gain(0.2);

    // On-axis scoring star
    let atm = AtmosphereBuilder::load("atmosphere/atmosphere.toml")?;
    let on_axis = OpticalModel::<NoSensor>::builder()
        .atmosphere(atm)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;

    println!("Model built in {}s", now.elapsed().as_secs());

    // SCOPES
    let mut monitor = Monitor::new();
    let scope_segment_piston = Scope::<SegmentPiston<-9>>::builder(&mut monitor).build()?;
    let scope_segment_wfe_rms = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor).build()?;
    let scope_wfe_rms = Scope::<WfeRms<-9>>::builder(&mut monitor).build()?;
    let scope_fsm_cmd = Scope::<M2FSMFsmCommand>::builder(&mut monitor).build()?;
    // ---

    // Bootstrapping the FEM and associated controls
    // let fem = state_space;
    let timer: Timer = Timer::new(4000);
    actorscript! {
        #[model(name=bootstrap)]
    1: timer[Tick] -> {servos::GmtFem}
    }

    let timer: Timer = Timer::new(n_step);
    type AgwsSh48 = Sh48<SH48_RATE>;
    type AgwsSh24 = Sh24<SH24_RATE>;
    type AgwsSh24Kernel = Kernel<Sh24<SH24_RATE>>;
    actorscript! {
        // #[model(state=running)]
    #[labels(on_axis = "On-axis Star",
         sh48_frame = "SH48\nframe")]//, sh24_frame = "SH24\nframe")]
    1: timer[Tick] -> {servos::GmtFem}

    // FEM state transfer to optical model
    1: {servos::GmtFem}[M1RigidBodyMotions]! -> {agws::AgwsSh48}
    1: {servos::GmtFem}[M2RigidBodyMotions]! -> {agws::AgwsSh48}
    1: {servos::GmtFem}[M1RigidBodyMotions]! -> {agws::AgwsSh24}
    1: {servos::GmtFem}[M2RigidBodyMotions]! -> {agws::AgwsSh24}
    1: {servos::GmtFem}[M1RigidBodyMotions]! -> on_axis
    1: {servos::GmtFem}[M2RigidBodyMotions]! -> on_axis

    // // AGWS SH24 to FSMS feedback loop
    100: {agws::AgwsSh24Kernel}[M2FSMFsmCommand] -> fsm_pzt_int
    1: fsm_pzt_int[M2FSMFsmCommand] -> {servos::GmtM2}

    5000: {agws::AgwsSh48}[Frame<Host>]! -> sh48_frame
    // 50: {agws::AgwsSh24}[Frame<Host>]! -> sh24_frame

    1: on_axis[WfeRms<-9>] -> scope_wfe_rms
    1: on_axis[SegmentWfeRms<-9>] -> scope_segment_wfe_rms
    1: on_axis[SegmentPiston<-9>] -> scope_segment_piston
    1000: on_axis[Wavefront]${512*512}
    }

    // model_logging_1000.lock().await.save();
    monitor.await?;

    Ok(())
}
