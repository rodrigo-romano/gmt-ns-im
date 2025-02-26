use std::time::Instant;

use gmt_dos_actors::actorscript;
use gmt_dos_actors_clients_interface::Tick;
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::Exponential};
use gmt_dos_clients_io::{
    gmt_m1::assembly,
    mount::{MountEncoders, MountTorques},
};
use gmt_dos_clients_m1_ctrl::Calibration;
use gmt_dos_clients_mount::Mount;
use gmt_dos_systems_m1::M1;
use gmt_fem::FEM;

const ACTUATOR_RATE: usize = 80;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
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
    let sim_duration = 1_usize; // second
    let n_step = sim_sampling_frequency * sim_duration;

    let mut fem = FEM::from_env()?;
    println!("{fem}");

    let m1_calibration = Calibration::new(&mut fem);

    // FEM MODEL
    let sids: Vec<u8> = vec![1, 2, 3, 4, 5, 6, 7];
    let state_space = DiscreteModalSolver::<Exponential>::from_fem(fem)
        .sampling(sim_sampling_frequency as f64)
        .proportional_damping(2. / 100.)
        .including_mount()
        .including_m1(Some(sids.clone()))?
        // .outs::<OSSM1Lcl>()
        // .outs::<MCM2Lcl6D>()
        .use_static_gain_compensation()
        .build()?;
    println!("{state_space}");

    // MOUNT CONTROL
    let mount = Mount::new();
    let m1 = M1::<ACTUATOR_RATE>::new(&m1_calibration)?;

    println!("Model built in {}s", now.elapsed().as_secs());

    let fem = state_space;
    let timer: Timer = Timer::new(n_step);
    actorscript! {
    // #[labels(fem = "GMT FEM", mount = "Mount\nControl", lom="Linear Optical\nModel")]
    1: timer[Tick] -> fem
    1:  mount[MountTorques] -> fem[MountEncoders]! -> mount

    1: {m1}[assembly::M1HardpointsForces]
        -> fem[assembly::M1HardpointsMotion]! -> {m1}
    1: {m1}[assembly::M1ActuatorAppliedForces] -> fem

    }

    Ok(())
}
