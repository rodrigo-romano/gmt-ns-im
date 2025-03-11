use std::{fs::File, time::Instant};

use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{gif, integrator::Integrator, signals::Signals, timer::Timer};
use gmt_dos_clients_crseo::{OpticalModel, calibration::Reconstructor, sensors::NoSensor};
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::Exponential};
use gmt_dos_clients_io::{
    gmt_fem::{
        inputs::{MCM2PZTF, MCM2SmHexF},
        outputs::{MCM2Lcl6D, MCM2PZTD, MCM2SmHexD, OSSM1EdgeSensors, OSSM1Lcl},
    },
    gmt_m1::{M1RigidBodyMotions, assembly},
    gmt_m2::{
        M2PositionerForces, M2PositionerNodes, M2RigidBodyMotions,
        fsm::{M2FSMFsmCommand, M2FSMPiezoForces, M2FSMPiezoNodes},
    },
    mount::{MountEncoders, MountTorques},
    optics::{Frame, Host, SegmentPiston, Wavefront, WfeRms},
};
use gmt_dos_clients_m1_ctrl::Calibration;
use gmt_dos_clients_m2_ctrl::Positioners;
use gmt_dos_clients_mount::Mount;
use gmt_dos_clients_scope::server::{Monitor, Scope};
use gmt_dos_systems_agws::{
    Agws,
    agws::{sh24::Sh24, sh48::Sh48},
    kernels::Kernel,
};
use gmt_dos_systems_m1::M1;
use gmt_dos_systems_m2::M2;
use gmt_fem::FEM;
use interface::Tick;

const ACTUATOR_RATE: usize = 10;
const SH48_RATE: usize = 5000;
const SH24_RATE: usize = 5000;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    println!("FEM  : {}", env!("FEM_REPO"));
    println!("MOUNT: {}", env!("MOUNT_MODEL"));
    // env::set_var(
    //     "DATA_REPO",
    //     Path::new(env!("CARGO_MANIFEST_DIR"))
    //         .join("src")
    //         .join("bin")
    //         .join("windloaded-mount-m1"),
    // );

    let now = Instant::now();

    let sim_sampling_frequency = 1000;
    let sim_duration = 10_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = FEM::from_env()?;
    // println!("{fem}");

    let m1_calibration = Calibration::new(&mut fem);

    // MOUNT CONTROL
    let mount = Mount::new();
    // M1 CONTROL
    assert_eq!(
        sim_sampling_frequency / ACTUATOR_RATE,
        100,
        "M1 actuators sampling rate is {} instead of 100Hz",
        sim_sampling_frequency / ACTUATOR_RATE
    );
    let m1 = M1::<ACTUATOR_RATE>::new(&m1_calibration)?;
    // M2 CONTROL
    let m2 = M2::new()?;
    // M2 POSITIONER CONTROL
    let m2_pos = Positioners::new(&mut fem)?;

    // FEM MODEL
    let sids: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7];
    let state_space = DiscreteModalSolver::<Exponential>::from_fem(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_mount()
        .including_m1(Some(sids.clone()))?
        .ins::<MCM2PZTF>()
        .ins::<MCM2SmHexF>()
        .outs::<MCM2PZTD>()
        .outs::<MCM2SmHexD>()
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .outs::<OSSM1EdgeSensors>()
        .use_static_gain_compensation()
        .build()?;
    println!("{state_space}");

    // AGWS
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/recon_sh24-to-rbm.pkl")?,
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
    // let fsm_pzt_int = Integrator::new(21).gain(0.5);

    // On-axis scoring star
    let on_axis = OpticalModel::<NoSensor>::builder().build()?;

    println!("Model built in {}s", now.elapsed().as_secs());

    // SCOPES
    let mut monitor = Monitor::new();
    let scope_segment_piston = Scope::<SegmentPiston<-9>>::builder(&mut monitor).build()?;
    let scope_wfe_rms = Scope::<WfeRms<-9>>::builder(&mut monitor).build()?;
    let scope_fsm_cmd = Scope::<M2FSMFsmCommand>::builder(&mut monitor).build()?;
    // ---

    let m2_rbm = Signals::new(42, n_step).channel(3, 1e-7);

    let fem = state_space;
    let timer: Timer = Timer::new(n_step);
    type AgwsSh48 = Sh48<SH48_RATE>;
    type AgwsSh24 = Sh24<SH24_RATE>;
    type AgwsSh24Kernel = Kernel<Sh24<SH24_RATE>>;
    actorscript! {
        // #[model(state=running)]
    // #[labels(fem = "GMT FEM", sh48_frame = "SH48\nframe", sh24_frame = "SH24\nframe")]
    1: timer[Tick] -> fem

    // Mount control
    1:  mount[MountTorques] -> fem[MountEncoders]! -> mount

    // M1 control
    1: {m1}[assembly::M1HardpointsForces]
        -> fem[assembly::M1HardpointsMotion]! -> {m1}
    1: {m1}[assembly::M1ActuatorAppliedForces] -> fem

    // M2 (positioner & FSMS) control
    1: m2_rbm[M2RigidBodyMotions]
        -> m2_pos[M2PositionerForces] -> fem[M2PositionerNodes]! -> m2_pos
    1: {m2}[M2FSMPiezoForces] -> fem[M2FSMPiezoNodes]! -> {m2}

    // FEM state transfer to optical model
    1: fem[M1RigidBodyMotions]! -> {agws::AgwsSh48}
    1: fem[M1RigidBodyMotions]! -> {agws::AgwsSh24}
    1: fem[M1RigidBodyMotions]! -> on_axis
    1: fem[M2RigidBodyMotions]! -> {agws::AgwsSh48}
    1: fem[M2RigidBodyMotions]! -> {agws::AgwsSh24}
    1: fem[M2RigidBodyMotions]! -> on_axis

    // AGWS SH24 to FSMS feedback loop
    5000: {agws::AgwsSh24Kernel}[M2FSMFsmCommand] -> scope_fsm_cmd //fsm_pzt_int
    // 1: fsm_pzt_int[M2FSMFsmCommand] -> {m2}

    5000: {agws::AgwsSh48}[Frame<Host>]! -> sh48_frame
    // 50: {agws::AgwsSh24}[Frame<Host>]! -> sh24_frame

    1: on_axis[WfeRms<-9>] -> scope_wfe_rms
    1: on_axis[SegmentPiston<-9>] -> scope_segment_piston
    1000: on_axis[Wavefront]${512*512}
    }

    model_logging_1000.lock().await.save();
    monitor.await?;

    Ok(())
}
