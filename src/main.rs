use std::{env, fs::File, path::Path, time::Instant};

use faer::{Mat, MatRef};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{gain::Gain, integrator::Integrator, timer::Timer};
use gmt_dos_clients_crseo::{
    calibration::{ClosedLoopCalib, Reconstructor},
    crseo::{FromBuilder, Gmt},
};
// use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::Exponential};
use gmt_dos_clients_io::{
    Estimate,
    gmt_m1::M1ModeShapes,
    gmt_m2::{
        M2RigidBodyMotions,
        fsm::{M2FSMFsmCommand, M2FSMPiezoNodes},
    },
};
use gmt_dos_clients_servos::{GmtFem, GmtM1, GmtM2, GmtM2Hex, GmtServoMechanisms, M1SegmentFigure};

use gmt_dos_clients_transceiver::{Monitor, Transceiver};
// use gmt_dos_clients_windloads::{
//     CfdLoads,
//     system::{M1, M2, Mount, SigmoidCfdLoads},
// };
use gmt_dos_systems_agws::{
    Agws,
    agws::{
        sh24::{Sh24, kernel::Sh24Kern},
        sh48::{Sh48, kernel::Sh48Kern},
    },
    builder::shack_hartmann::ShackHartmannBuilder,
    qp::{ActiveOptics, Estimate2OpticsState, QP},
};
use gmt_dos_systems_m1::SingularModes;
use gmt_fem::FEM;
use interface::{
    Tick,
    optics::{
        M1State, OpticsState,
        state::{MirrorState, OpticalState},
    },
};
use matio_rs::MatFile;

const N_MODE: usize = 271;
const M1_BM: usize = 27;
const M1_RBM: usize = 41;
const M2_RBM: usize = 41;

#[cfg(not(feature = "qp"))]
type K48 = Sh48<{ config::agws::sh48::RATE }>;
#[cfg(feature = "qp")]
type K48 = ActiveOptics<{ config::agws::sh48::RATE }, 41, 41, 27, 271>;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();
    println!("FEM  : {}", env!("FEM_REPO"));
    println!("MOUNT: {}", env!("MOUNT_MODEL"));

    let now = Instant::now();

    let sim_sampling_frequency = 1000;
    let sim_duration = 60_usize; // second
    let bootstrapping_duration = 4_usize; // second
    let n_bootstrapping = sim_sampling_frequency * bootstrapping_duration;
    let n_sim = sim_sampling_frequency * sim_duration + 1;

    let fem = FEM::from_env()?;
    // println!("{fem}");

    // let cfd_loads = Sys::<SigmoidCfdLoads>::try_from(
    //     CfdLoads::foh(".", sim_sampling_frequency)
    //         .duration((sim_duration + bootstrapping_duration) as f64)
    //         .mount(&mut fem, 0, None)
    //         .m1_segments()
    //         .m2_segments(),
    // )?;

    // M1 EDGE SENSORS TO RIGID-BODY MOTIONS TRANSFORM
    // let m1_es_2_rbm: nalgebra::DMatrix<f64> =
    //     MatFile::load("calibrations/m1/edge-sensors/es_2_rbm.mat")?.var("m1_r_es")?;
    // let servos =
    //     Sys::<GmtServoMechanisms<{ config::m1::ACTUATOR_RATE }, 1>>::from_data_repo_or_else(
    //         "servos.bin",
    //         || {
    //             GmtServoMechanisms::<{ config::m1::ACTUATOR_RATE }, 1>::new(
    //                 sim_sampling_frequency as f64,
    //                 fem,
    //             )
    //             .m1_segment_figure(M1SegmentFigure::new())
    //         },
    //     )?;

    // SERVO-MECHANISMS
    // let mut m1s1_modes0 = vec![0f64; config::m1::segment::N_RAW_MODE];
    // m1s1_modes0[0] = 1e-6;
    let servos = {
        let m1_sms: SingularModes = serde_pickle::from_reader(
            &File::open("calibrations/m1/modes/m1_singular_modes.pkl")?,
            Default::default(),
        )?;

        let b2f: Vec<_> = m1_sms
            .mode2force()
            .into_iter()
            .map(|mat| mat.columns(0, config::m1::segment::N_MODE).clone_owned())
            .collect();
        let s2b: Vec<_> = m1_sms
            .raw_modes_into_mat()
            .into_iter()
            .map(|x| x.transpose())
            .collect();

        GmtServoMechanisms::<{ config::m1::segment::ACTUATOR_RATE }, 1>::new(
            sim_sampling_frequency as f64,
            fem,
        )
        // .wind_loads(WindLoads::new())
        .m1_segment_figure(M1SegmentFigure::new().transforms(s2b).modes_to_forces(b2f))
        .build()?
    };
    println!("{servos}");
    // serde_pickle::to_writer(
    //     &mut File::create("servos.bin")?,
    //     &servos,
    //     Default::default(),
    // )?;
    // let rdr = BufReader::new(File::open("servos.bin")?);
    // let servos :Sys< GmtServoMechanisms<{ config::m1::ACTUATOR_RATE }, 1>>=
    // serde_pickle::from_reader(rdr, Default::default())?;

    // AGWS
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh24/recon_sh24-to-pzt_pth.pkl")?,
        Default::default(),
    )?;
    let m1_bm_recon: Reconstructor<_, ClosedLoopCalib> = serde_pickle::from_reader(
        File::open("calibrations/sh48/closed_loop_recon_sh48-to-m1-bm.pkl")?,
        Default::default(),
    )?;
    println!("SH24 to FSM reconstructor:\n{recon}");
    println!("closed-loop SH48 to M1 BM reconstructor:\n{m1_bm_recon}");

    let data_path = Path::new("/home/ubuntu/projects/im-sim-scripts/aco_loop_example/data");
    let mut aco = QP::<M1_RBM, M2_RBM, 27, N_MODE>::new(
        //"../aco_impl_stdalone/SHAcO_qp_rhoP1e-3_kIp5.rs.pkl")
        //"rustCalib_AcO_rhoP1e-12_kIp5.rs.pkl")
        data_path.join("rustCalib_AcO_rhoP1e-12_kIp5.agws.pickle"),
    )?
    .update_calib(
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("qp")
            .join("sh48_calibration.pkl"),
    )?
    .build()?;
    println!("{aco}");
    aco.set_controller_gain(0f64);

    let (agws_wss, mut agws): (
        _,
        Sys<
            Agws<
                { config::agws::sh48::RATE },
                { config::agws::sh24::RATE },
                K48,
                Sh24<{ config::agws::sh24::RATE }>,
            >,
        >,
    ) = {
        let agws = if config::ATMOSPHERE {
            Agws::<
                { config::agws::sh48::RATE },
                { config::agws::sh24::RATE },
                K48,
                Sh24<{ config::agws::sh24::RATE }>,
            >::builder()
            .load_atmosphere("atmosphere/atmosphere.toml", sim_sampling_frequency as f64)?
        } else {
            Agws::<
                { config::agws::sh48::RATE },
                { config::agws::sh24::RATE },
                K48,
                Sh24<{ config::agws::sh24::RATE }>,
            >::builder()
            .sh24(
                ShackHartmannBuilder::sh24()
                    // .source(AgwsGuideStar::sh24().zenith_azimuth(vec![0f32], vec![0f32]))
                    .use_calibration_src(),
            )
            .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
        }
        .gmt(Gmt::builder().m1(
            config::m1::segment::RAW_MODES,
            config::m1::segment::N_RAW_MODE,
        ))
        .sh24_calibration(recon);
        #[cfg(not(feature = "qp"))]
        let agws = agws.sh48_calibration(m1_bm_recon);
        #[cfg(feature = "qp")]
        let agws = agws.sh48_calibration(aco);
        (agws.wave_sensor().build()?, agws.build()?)
    };
    if let Some(p24) = config::agws::sh24::POINTING_ERROR {
        let _ = agws.sh24_pointing(p24).await;
    }
    println!("{agws}");
    println!("{agws_wss}");

    // M1 edge sensors to RBMs integrator
    // let m1_es_to_rbm_int = Integrator::new(42).gain(config::m1::edge_sensor::RBM_INTEGRATOR_GAIN);

    /* // Mount reconstructor
    let mount_recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/mount/recon_sh48-to-mount.pkl")?,
        Default::default(),
    )?;
    println!("SH48 to Mount reconstructor:\n{mount_recon}");

    // M1 assembly tip-tilt reconstructor
    let m1_recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/m1/assembly/recon_sh48-to-m1-assembly.pkl")?,
        Default::default(),
    )?;
    println!("SH48 to Mount reconstructor:\n{m1_recon}"); */

    println!("Model built in {}s", now.elapsed().as_secs());

    // PERTURBATIONS
    // let m2_rbm = Signals::new(42, 3000 + n_bootstrapping).channel(3, 1e-6);
    // let mount_cmd = Signals::new(3, n_sim); //.channel(0, -1e-5).channel(1, 1e-5);
    // let mut m1_rbm = vec![vec![0f64; 6]; 7];
    // m1_rbm[0][0] = 1e-6;
    // m1_rbm[6][4] = 1e-6;
    // let m1_rbm = Signals::from((m1_rbm, n_sim));
    // let mut m2_rbm = vec![vec![0f64; 6]; 7];
    // m2_rbm[0][0] = 1e-6;
    // m2_rbm[0][3] = 1e-6;
    // m2_rbm[1][2] = 1e-6;
    // m2_rbm[1][2] = 1e-6;
    // m2_rbm[4][1] = 1e-6;
    // m2_rbm[6][1] = 1e-6;
    // let m2_rbm = Signals::from((m2_rbm, n_sim));
    // // let adder = Operator::new("+");
    // let m2_adder = Operator::<Vec<f64>>::plus();
    // Bootstrapping the FEM and associated controls
    // let fem = state_space;

    // let matfile = MatFile::load("calibrations/m1/modes/20230530_1756_m1_mode_to_force.mat")?;
    // let b2f: Vec<Mat<f64>> = (1..=7)
    //     .map(|i| matfile.var(format!("B2F_{i}")).unwrap())
    //     .collect();
    // println!(
    //     "B2F: {:?}",
    //     b2f.iter().map(|x| x.shape()).collect::<Vec<_>>()
    // );
    // let m1_bm_2_forces = Gain::<f64>::new(
    //     b2f.iter()
    //         .map(|x| x.subcols(0, config::m1::segment::N_MODE).to_owned())
    //         .collect::<Vec<_>>(),
    // );
    // let mut m1_bm = vec![vec![0f64; config::m1::segment::N_MODE]; 7];
    // // m1_bm[0][0] = 1e-6;
    // m1_bm
    //     .iter_mut()
    //     .enumerate()
    //     // .skip(2)
    //     // .take(1)
    //     .for_each(|(i, b)| b[0] = 1e-6);
    // let m1_bm = Signals::from((m1_bm, n_sim));
    // let m1_bms = M1BendingModes::new("calibrations/m1/modes/m1_singular_modes.pkl")?;
    let mut timer: Timer = Timer::new(n_bootstrapping);
    timer.progress();

    // let state_print = gmt_dos_clients::print::Print::new(8);

    let address = "127.0.0.1";
    let mut gmt_state_mon = Monitor::new();
    let gmt_state_tx = Transceiver::<OpticsState>::transmitter(address)?.run(&mut gmt_state_mon);
    actorscript! {
        #[model(name=bootstrap)]
    1: timer[Tick] -> {servos::GmtFem}

    // 1: {cfd_loads::M1}[CFDM1WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::M2}[CFDM2WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::Mount}[CFDMountWindLoads] -> {servos::GmtFem}

    // 1: mount_cmd[MountSetPoint] -> {servos::GmtMount}
    // 1: m1_rbm[M1RigidBodyMotions] -> {servos::GmtM1}
    // 1: m2_rbm[M2RigidBodyMotions] -> {servos::GmtM2Hex}
    // 1: m1_bm[M1ModeShapes] -> m1_bm_2_forces[M1ActuatorCommandForces] -> {servos::GmtM1}
    // 1: {servos::GmtFem}[M1State] -> on_axis
    1: {servos::GmtFem}[OpticsState].. -> gmt_state_tx
    // 1: {servos::GmtFem}[OpticsState] -> state_print

    // 1: {servos::GmtFem}[M1EdgeSensors]

    }

    // M2 RBM SH48 calibration
    let sh48_m2_rbm_recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh48/open_loop_recon_sh48-to-m2-rbm.pkl")?,
        Default::default(),
    )?;
    println!("SH48 to M2 RBM reconstructor:\n{sh48_m2_rbm_recon}");
    // let pol = PseudoOpenLoop::new(sh48_m2_rbm_recon);
    // M1 RBM SH48 calibration
    // let sh48_m1_rbm_recon: Reconstructor = serde_pickle::from_reader(
    //     File::open("calibrations/sh48/open_loop_recon_sh48-to-m1-rxy.pkl")?,
    //     Default::default(),
    // )?;
    // println!("SH48 to M1 RBM reconstructor:\n{sh48_m1_rbm_recon}");
    // // let s1 = Sampler::default();
    // let s2 = Sampler::default();

    // M1 BM SH48 calibration
    // let sh48_m1_bm_recon: Reconstructor<_, ClosedLoopCalib> = serde_pickle::from_reader(
    //     File::open("calibrations/sh48/closed_loop_recon_sh48-to-m1-bm.pkl")?,
    //     Default::default(),
    // )?;
    // let m1_bm_adder = Operator::<Vec<f64>>::plus();
    // let sh48_int = Integrator::new(27 * 7).gain(0.5);

    // let sh48_m2_rbm_recon: Reconstructor<_, ClosedLoopCalib> = serde_pickle::from_reader(
    //     File::open("calibrations/sh48/closed_loop_recon_sh48-to-m2-rbm.pkl")?,
    //     Default::default(),
    // )?;
    // println!("SH48 M2 RBM\n{sh48_m1_rbm_recon}");
    // let mut sh48_m2_rbm_m1_bm_recon: MergeReconstructor<_, M1RigidBodyMotions, _> =
    //     MergeReconstructor::new(
    //         // "calibrations/sh48/closed_loop_recon_sh48-to-m2-rbm.pkl",
    //         "calibrations/sh48/e_1_48.pkl",
    //         "calibrations/sh48/closed_loop_recon_sh48-to-m1-bm.pkl",
    //         None,
    //     )?;
    // // sh48_m2_rbm_recon.truncated_pseudoinverse(vec![1   // sh48_m2_r
    // // bm_rrecon.truncated_p
    // // seudoinverse(vec![1, 1, 1, 1, 1, 1, 0]);
    // println!("CLOSED LOOP SH48 M2 RBM & M1 BM {sh48_m2_rbm_m1_bm_recon}");
    // let m2_rbm_adder = Operator::<Vec<f64>>::plus();

    // let lpf = LowPassFilter::new(42, 2e-3);

    // let m1_bm_recon: Reconstructor<_, ClosedLoopCalib> =
    //     Reconstructor::from_path("calibrations/sh48/closed_loop_recon_sh48-to-m1-bm.pkl")?;

    dbg!(&timer.lock().await);

    // FSM OFF-LOAD TO POSITIONER
    let matfile = MatFile::load("calibrations/sh24/m2_pzt_r.mat")?;
    let pzt_to_rbm: Vec<Mat<f64>> = (0..7)
        .map(|i| {
            let var: Vec<f64> = matfile.var(format!("var{i}")).unwrap();
            let mat = MatRef::from_column_major_slice(&var, 6, 6);
            mat.to_owned()
        })
        .collect();
    let pzt_to_rbm = Gain::<f64>::new(pzt_to_rbm);
    // FSM off-load integrator
    let pzt_to_rbm_int = Integrator::new(42).gain(config::fsm::OFFLOAD_INTEGRATOR_GAIN);

    // FSM command integrator
    let fsm_pzt_int = Integrator::new(21).gain(config::agws::sh24::INTEGRATOR_GAIN);

    // let print = Print::<Vec<f64>>::new(8);
    let timer: Timer = Timer::new(n_sim);
    type AgwsSh48 = Sh48<{ config::agws::sh48::RATE }>;
    type AgwsSh24 = Sh24<{ config::agws::sh24::RATE }>;
    type AgwsSh24Kernel = Sh24Kern<Sh24<{ config::agws::sh24::RATE }>>;
    type AgwsSh48Kernel = Sh48Kern<K48>;
    // let one_to_1000 = Sampler::default();
    let e2o = Estimate2OpticsState::new();

    let mut m1_rbm = vec![vec![0f64; 6]; 7];
    m1_rbm[0][0] = 1. * 1.1e-6; // M1S1-Tx:
    // m1_rbm[1][1] = 1. * 1.2e-6; // M1S2-Ty:
    // m1_rbm[2][3] = 1. * 1.4e-6; // M1S3-Rx:
    // m1_rbm[3][4] = 1. * 1.5e-6; // M1S4-Ry:
    // m1_rbm[4][2] = 1. * 1.6e-6; // M1S5-Tz:
    // m1_rbm[5][5] = 1. * 1.3e-6; // M1S5-Rz:
    // m1_rbm[6][5] = 1. * 2e-6; // M1S7-Rz:
    let mut m2_rbm = vec![0f64; 42];
    // m2_rbm[M2_RBM] = 1. * 3e-4; // M2S7-Rz
    let mut m1_modes = vec![vec![0f64; M1_BM]; 7];
    // m1_modes[0][0] = 4e-6;
    // m1_modes[0][2] = 5e-6;
    let m1 = MirrorState::new(m1_rbm, m1_modes);
    let zero_point = OpticalState::new(m1, MirrorState::from_rbms(&m2_rbm));
    let optical_state = OpticalState::default().zero_point(zero_point);
    // dbg!(&optical_state);
    // let state_print = gmt_dos_clients::print::Print::default().scale(1e9_f64);
    actorscript! {
        // #[model(state=running)]
    #[labels(//on_axis = "GMT Optics & Atmosphere\nw/ On-Axis Star",
         // mount_cmd="Mount Set-Point",
          // m1_rbm="M1 RBM",
          // m2_rbm="M2 RBM",
         // m1_bm="M1 BM",
         // m1_bm_recon="SH48\nM1 BM\nReconstructor",
         // m1_bm_2_forces="Mode to Force",
         fsm_pzt_int="FSM\nIntegrator",
         pzt_to_rbm="FSM\nto\nPositioner",
         pzt_to_rbm_int="Positioner\nIntegrator",
         // m1_es_to_rbm_int="M1 RBM\nIntegrator",
         // adder="Adder",
         // m2_adder="Adder",
         // m2_rbm_adder="Substracter",
         // m1_bm_adder="Adder",//s2="1:1000",
         // sh48_int="M1 BM\nIntegrator",
         // gmt_state_tx="Beam me up, Scotty"
         gmt_state_tx="🕪"
         )]
    1: timer[Tick] -> {servos::GmtFem}

    // 1: {cfd_loads::M1}[C10_DM1WindLoads] -AgwsSh48Kernel> SensorDa${cfd_loads::M2}[CFDM2WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::Mount}[CFDMountWindLoads] -> {servos::GmtFem}

    // 1: mount_cmd[MountSetPoint] -> {servos::GmtMount}
    // 1: m1_rbm[Left<M1RigidBodyMotions>] -> adder[M1RigidBodyMotions] -> {servos::GmtM1}
    // 5000: m2_rbm[Left<M2RigidBodyMotions>] -> m2_adder
    // 5000: m1_bm[Left<M1ModeShapes>] -> m1_bm_adder[M1ModeShapes]  -> m1_bm_2_forces
    // 1: m1_bm_2_forces[M1ActuatorCommandForces] -> {servos::GmtM1}

    1:  {servos::GmtFem}[OpticsState]! -> {agws::AgwsSh24}
    1:  {servos::GmtFem}[OpticsState]! -> {agws::AgwsSh48}

    1: {servos::GmtFem}[OpticsState]!.. -> gmt_state_tx
    // 1: {servos::GmtFem}[M1State]!.. -> state_print

    // 1: {servos::GmtFem}[Mas<AverageMountEncoders>] -> mount_scopes

    // FSM to positionner off-load
    // 1: {servos::GmtFem}[M2FSMPiezoNodes]
    //     -> pzt_to_rbm[Left<M2RigidBodyMotions>] //-> scope_fsm_cmd
    //     // 5000: m2_rbm_adder[M2RigidBodyMotions]${42}
    //         5000: pzt_to_rbm_int[Right<M2RigidBodyMotions>]
    //         -> m2_adder//[M2RigidBodyMotions]${42}
    //            // -> lpf
    // 1: m2_adder[M2RigidBodyMotions] -> {servos::GmtM2Hex}
    1: {servos::GmtFem}[M2PositionerNodes]
    1: {servos::GmtFem}[M2FSMPiezoNodes]
        -> pzt_to_rbm[M2RigidBodyMotions] //-> scope_fsm_cmd
            -> pzt_to_rbm_int[M2RigidBodyMotions]
                -> {servos::GmtM2Hex}

    // M1 edge sensor to RBMs feedback loop
    // 1: {servos::GmtFem}[M1EdgeSensors]!
    //     -> m1_es_to_rbm_int[M1RigidBodyMotions]
    //         -> {servos::GmtM1}
            // -> adder


    // // AGWS SH24 to FSMS feedback loop
    5: {agws::AgwsSh24Kernel}[M2FSMFsmCommand] -> fsm_pzt_int
    1: fsm_pzt_int[M2FSMFsmCommand] -> {servos::GmtM2}

    // 5000: {agws::AgwsSh48Kernel}[SensorData] -> m1_bm_recon
    // 5000: sh48_m2_rbm_m1_bm_recon[SplitEstimate<0>]${42} -> pzt_to_rbm_int
    //     // -> m2_rbm_adder
    // 5000: sh48_m2_rbm_m1_bm_recon[SplitEstimate<1>]${27*7}
    //     -> sh48_int[Right<Estimate>] -> m1_bm_adder
    // 5000: m1_bm_recon[Estimate]${27*7}
    5000: {agws::AgwsSh48Kernel}[OpticsState]-> optical_state  // -> sh48_int
    // 1: optical_state[M1State] -> state_print
    1: optical_state[M1State] -> {servos::GmtM1}
    // 1: sh48_int[M1ModeShapes] -> {servos::GmtM1}
    // 1000: {agws::AgwsSh48Kernel}[SensorData] -> mount_recon[MountEstimate] -> print
    // // 1000: {agws::AgwsSh48Kernel}[SensorData] -> pol//m1_recon//[Estimate] -> print
    // 1000: pzt_to_rbm[M2RigidBodyMotions]
    // //          -> pol[PseudoSensorData] -> mount_recon[Estimate]->print

    }

    gmt_state_mon.await?;
    Ok(())
}
