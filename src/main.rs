use std::{fs::File, time::Instant};

use faer::{Mat, MatRef};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    gain::Gain,
    gif,
    integrator::Integrator,
    low_pass_filter::LowPassFilter,
    operator::{Left, Operator, Right},
    print::Print,
    sampler::Sampler,
    signals::Signals,
    timer::Timer,
};
use gmt_dos_clients_crseo::{
    OpticalModel,
    calibration::{ClosedLoopCalib, Reconstructor},
    crseo::{FromBuilder, Gmt, builders::AtmosphereBuilder},
    sensors::NoSensor,
};
// use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::Exponential};
use gmt_dos_clients_io::{
    Estimate,
    gmt_m1::{M1EdgeSensors, M1ModeShapes, M1RigidBodyMotions, assembly::M1ActuatorCommandForces},
    gmt_m2::{
        M2RigidBodyMotions,
        fsm::{M2FSMFsmCommand, M2FSMPiezoNodes},
    },
    mount::{AverageMountEncoders, MountSetPoint},
    optics::{
        M1State, M2State, SegmentPiston, SegmentTipTilt, SegmentWfeRms, SensorData, TipTilt,
        Wavefront, WfeRms,
    },
};
use gmt_dos_clients_lom::LinearOpticalModel;
use gmt_dos_clients_servos::{
    EdgeSensors, GmtFem, GmtM1, GmtM2, GmtM2Hex, GmtMount, GmtServoMechanisms, M1SegmentFigure,
};
// use gmt_dos_clients_windloads::{
//     CfdLoads,
//     system::{M1, M2, Mount, SigmoidCfdLoads},
// };
use gmt_dos_systems_agws::{
    Agws,
    agws::{sh24::Sh24, sh48::Sh48},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::Kernel,
};
use gmt_fem::FEM;
use gmt_ns_im::{
    MergeReconstructor, SplitEstimate, config, m1_bending_modes::M1BendingModes, scopes::*,
};
use interface::{Tick, units::Mas};
use matio_rs::MatFile;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    println!("FEM  : {}", env!("FEM_REPO"));
    println!("MOUNT: {}", env!("MOUNT_MODEL"));

    let now = Instant::now();

    let sim_sampling_frequency = 1000;
    let sim_duration = 80_usize; // second
    let bootstrapping_duration = 4_usize; // second
    let n_bootstrapping = sim_sampling_frequency * bootstrapping_duration;
    let n_sim = n_bootstrapping + sim_sampling_frequency * sim_duration + 1;

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
    let m1_es_2_rbm: nalgebra::DMatrix<f64> =
        MatFile::load("calibrations/m1/edge-sensors/es_2_rbm.mat")?.var("m1_r_es")?;
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

    let servos = GmtServoMechanisms::<{ config::m1::segment::ACTUATOR_RATE }, 1>::new(
        sim_sampling_frequency as f64,
        fem,
    )
    .edge_sensors(EdgeSensors::m1().m1_with(m1_es_2_rbm))
    // .wind_loads(WindLoads::new())
    .m1_segment_figure(M1SegmentFigure::new())
    .build()?;
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
    println!("SH24 to FSM reconstructor:\n{recon}");
    let (agws_wss, mut agws): (
        _,
        Sys<Agws<{ config::agws::sh48::RATE }, { config::agws::sh24::RATE }>>,
    ) = {
        let agws = if config::ATMOSPHERE {
            Agws::builder()
                .load_atmosphere("atmosphere/atmosphere.toml", sim_sampling_frequency as f64)?
        } else {
            Agws::builder()
                .sh24(
                    ShackHartmannBuilder::sh24()
                        // .source(AgwsGuideStar::sh24().zenith_azimuth(vec![0f32], vec![0f32]))
                        .use_calibration_src(),
                )
                .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
        }
        .gmt(Gmt::builder().m1(
            gmt_ns_im::config::m1::segment::RAW_MODES,
            gmt_ns_im::config::m1::segment::N_RAW_MODE,
        ))
        .sh24_calibration(recon);
        (agws.wave_sensor().build()?, agws.build()?)
    };
    if let Some(p24) = config::agws::sh24::POINTING_ERROR {
        let _ = agws.sh24_pointing(p24).await;
    }
    println!("{agws}");
    println!("{agws_wss}");

    // let sh48_frame: gif::Frame<f32> = gif::Frame::new("sh48_frame.png", 48 * 8);
    // let sh24_frame: gif::Frame<f32> = gif::Frame::new("sh24_frame.png", 24 * 12);
    // let on_axis_wavefront: gif::Frame<f64> = gif::Frame::new("on-axis_wavefront.png", 512);
    let agws_wavefronts: gif::Frame<f64> = gif::Frame::new("agws_wavefronts.png", 512);
    let on_axis_wavefront: gif::Gif<f64> =
        gif::Gif::new("on-axis_wavefront.gif", 512, 512)?.delay(200);

    // FSM command integrator
    let fsm_pzt_int = Integrator::new(21).gain(config::agws::sh24::INTEGRATOR_GAIN);

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

    // M1 edge sensors to RBMs integrator
    let m1_es_to_rbm_int = Integrator::new(42).gain(config::m1::edge_sensor::RBM_INTEGRATOR_GAIN);

    // On-axis scoring star
    let atm = AtmosphereBuilder::load("atmosphere/atmosphere.toml")?;
    let on_axis = if config::ATMOSPHERE {
        OpticalModel::<NoSensor>::builder().atmosphere(atm)
    } else {
        OpticalModel::<NoSensor>::builder()
    }
    .gmt(Gmt::builder().m1(
        config::m1::segment::RAW_MODES,
        config::m1::segment::N_RAW_MODE,
    ))
    .sampling_frequency(sim_sampling_frequency as f64)
    .build()?;

    // Linear Optical Models
    let m1_lom = LinearOpticalModel::new()?;
    let m2_lom = LinearOpticalModel::new()?;

    // Mount reconstructor
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
    println!("SH48 to Mount reconstructor:\n{m1_recon}");

    println!("Model built in {}s", now.elapsed().as_secs());

    // SCOPES
    let shub = OnAxisScopes::new()?;
    let mount_scopes = MountScopes::new()?;

    let m1_scopes = M1Scopes::new()?;
    let m2_scopes = M2Scopes::new()?;
    // ---

    // PERTURBATIONS
    // let m2_rbm = Signals::new(42, 3000 + n_bootstrapping).channel(3, 1e-6);
    let mount_cmd = Signals::new(3, n_sim);
    // .channel(0, -1e-5)
    // .channel(1, 1e-5);
    let mut m1_rbm = vec![vec![0f64; 6]; 7];
    // m1_rbm[0][0] = 1e-6;
    // m1_rbm[6][4] = 1e-6;
    let m1_rbm = Signals::from((m1_rbm, n_sim));
    let mut m2_rbm = vec![vec![0f64; 6]; 7];
    m2_rbm[0][0] = 1e-6;
    // m2_rbm[6][4] = 1e-6;
    let m2_rbm = Signals::from((m2_rbm, n_sim));
    let adder = Operator::new("+");
    let m2_adder = Operator::<f64>::new("+");
    // Bootstrapping the FEM and associated controls
    // let fem = state_space;

    let matfile = MatFile::load("calibrations/m1/modes/20230530_1756_m1_mode_to_force.mat")?;
    let b2f: Vec<Mat<f64>> = (1..=7)
        .map(|i| matfile.var(format!("B2F_{i}")).unwrap())
        .collect();
    println!(
        "B2F: {:?}",
        b2f.iter().map(|x| x.shape()).collect::<Vec<_>>()
    );
    let m1_bm_2_forces = Gain::<f64>::new(
        b2f.iter()
            .map(|x| x.subcols(0, config::m1::segment::N_MODE).to_owned())
            .collect::<Vec<_>>(),
    );
    let mut m1_bm = vec![vec![0f64; config::m1::segment::N_MODE]; 7];
    // m1_bm
    //     .iter_mut()
    //     .enumerate()
    //     .take(1)
    //     .for_each(|(i, b)| b[i] = 1e-6);
    let m1_bm = Signals::from((m1_bm, n_sim));
    let m1_bms = M1BendingModes::new("calibrations/m1/modes/m1_singular_modes.pkl")?;

    let timer: Timer = Timer::new(n_bootstrapping);
    actorscript! {
        #[model(name=bootstrap)]
    1: timer[Tick] -> {servos::GmtFem}

    // 1: {cfd_loads::M1}[CFDM1WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::M2}[CFDM2WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::Mount}[CFDMountWindLoads] -> {servos::GmtFem}

    1: mount_cmd[MountSetPoint] -> {servos::GmtMount}
    1: m1_rbm[M1RigidBodyMotions] -> {servos::GmtM1}
    1: m2_rbm[M2RigidBodyMotions] -> {servos::GmtM2Hex}
    1: m1_bm[M1ModeShapes] -> m1_bm_2_forces[M1ActuatorCommandForces] -> {servos::GmtM1}
    1: {servos::GmtFem}[M1State] -> m1_bms[M1State] -> on_axis

    // 1: {servos::GmtFem}[M1RigidBodyMotions] -> on_axis
    1: {servos::GmtFem}[M2State] -> on_axis
    1: {servos::GmtFem}[M1RigidBodyMotions] -> m1_lom
    1: {servos::GmtFem}[M2RigidBodyMotions] -> m2_lom
    1: m1_lom[M1SegmentPiston].. -> m1_scopes
    1: m2_lom[M2SegmentPiston].. -> m2_scopes
    1: m1_lom[M1SegmentTipTilt].. -> m1_scopes
    1: m2_lom[M2SegmentTipTilt].. -> m2_scopes

    // 1: {servos::GmtFem}[M1EdgeSensors]

    1: on_axis[WfeRms<-9>].. -> shub
    1: on_axis[SegmentWfeRms<-9>].. -> shub
    1: on_axis[Mas<TipTilt>].. -> shub
    1: on_axis[Mas<SegmentTipTilt>].. -> shub
    1: on_axis[SegmentPiston<-9>] -> shub
    1000: on_axis[Wavefront].. -> on_axis_wavefront
    }

    // M2 RBM SH48 calibration
    let sh48_m2_rbm_recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh48/open_loop_recon_sh48-to-m2-rbm.pkl")?,
        Default::default(),
    )?;
    println!("SH48 to M2 RBM reconstructor:\n{sh48_m2_rbm_recon}");
    // let pol = PseudoOpenLoop::new(sh48_m2_rbm_recon);
    // M1 RBM SH48 calibration
    let sh48_m1_rbm_recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh48/open_loop_recon_sh48-to-m1-rxy.pkl")?,
        Default::default(),
    )?;
    println!("SH48 to M1 RBM reconstructor:\n{sh48_m1_rbm_recon}");
    // let s1 = Sampler::default();
    let s2 = Sampler::default();

    // M1 BM SH48 calibration
    let sh48_m1_bm_recon: Reconstructor<_, ClosedLoopCalib> = serde_pickle::from_reader(
        File::open("calibrations/sh48/closed_loop_recon_sh48-to-m1-bm.pkl")?,
        Default::default(),
    )?;
    let m1_bm_adder = Operator::<f64>::new("+");
    let sh48_int = Integrator::new(27 * 7).gain(0.1);

    // let sh48_m2_rbm_recon: Reconstructor<_, ClosedLoopCalib> = serde_pickle::from_reader(
    //     File::open("calibrations/sh48/closed_loop_recon_sh48-to-m2-rbm.pkl")?,
    //     Default::default(),
    // )?;
    println!("SH48 M2 RBM\n{sh48_m1_rbm_recon}");
    let mut sh48_m2_rbm_m1_bm_recon = MergeReconstructor::new(
        "calibrations/sh48/closed_loop_recon_sh48-to-m2-rbm.pkl",
        "calibrations/sh48/closed_loop_recon_sh48-to-m1-bm.pkl",
        None,
    )?;
    // sh48_m2_rbm_recon.truncated_pseudoinverse(vec![1   // sh48_m2_r
    // bm_rrecon.truncated_p
    // seudoinverse(vec![1, 1, 1, 1, 1, 1, 0]);
    println!("CLOSED LOOP SH48 M2 RBM & M1 BM {sh48_m2_rbm_m1_bm_recon}");
    let m2_rbm_adder = Operator::<f64>::new("+");

    let lpf = LowPassFilter::new(42, 2e-3);

    // let print = Print::<Vec<f64>>::new(8);
    // let timer: Timer = Timer::new(6001n_sim
    type AgwsSh48 = Sh48<{ config::agws::sh48::RATE }>;
    type AgwsSh24 = Sh24<{ config::agws::sh24::RATE }>;
    type AgwsSh24Kernel = Kernel<Sh24<{ config::agws::sh24::RATE }>>;
    type AgwsSh48Kernel = Kernel<Sh48<{ config::agws::sh48::RATE }>>;
    actorscript! {
        // #[model(state=running)]
    #[labels(on_axis = "GMT Optics & Atmosphere\nw/ On-Axis Star",
         mount_cmd="Mount Set-Point",
          m1_rbm="M1 RBM",
          m2_rbm="M2 RBM",
         m1_bm="M1 BM",
         sh48_m2_rbm_m1_bm_recon="SH48\nM2 RBM & M1 BM\nReconstructor",
         m1_bm_2_forces="Mode to Force",
         fsm_pzt_int="FSM\nIntegrator",
         // pzt_to_rbm="FSM\nto\nPositioner",
         pzt_to_rbm_int="Positioner\nIntegrator",
         m1_es_to_rbm_int="M1 RBM\nIntegrator",
         adder="Adder",
         m2_adder="Adder",
         // m2_rbm_adder="Substracter",
         m1_bm_adder="Adder",s2="1:1000",
         sh48_int="M1 BM\nIntegrator"
         )]
    // 1: timer[Tick] -> {servos::GmtFem}

    // 1: {cfd_loads::M1}[C10_DM1WindLoads] -AgwsSh48Kernel> SensorDa${cfd_loads::M2}[CFDM2WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::Mount}[CFDMountWindLoads] -> {servos::GmtFem}

    1: mount_cmd[MountSetPoint] -> {servos::GmtMount}
    1: m1_rbm[Left<M1RigidBodyMotions>] -> adder[M1RigidBodyMotions] -> {servos::GmtM1}
    5000: m2_rbm[Left<M2RigidBodyMotions>] -> m2_adder
    5000: m1_bm[Left<M1ModeShapes>] -> m1_bm_adder[M1ModeShapes]  -> m1_bm_2_forces
    1: m1_bm_2_forces[M1ActuatorCommandForces] -> {servos::GmtM1}
    1: {servos::GmtFem}[M1State]
        -> m1_bms[M1State] -> on_axis
    1000:  m1_bms[M1State] -> agws_wss
    1:  m1_bms[M1State] -> {agws::AgwsSh48}
    1:  m1_bms[M1State] -> {agws::AgwsSh24}

    1: {servos::GmtFem}[Mas<AverageMountEncoders>] -> mount_scopes

    // FSM to positionner off-load
    // 1: {servos::GmtFem}[M2FSMPiezoNodes]
        // -> pzt_to_rbm[Left<M2RigidBodyMotions>] //-> scope_fsm_cmd
        // 5000: m2_rbm_adder[M2RigidBodyMotions]${42}
            5000: pzt_to_rbm_int[Right<M2RigidBodyMotions>]
            -> m2_adder[M2RigidBodyMotions]${42}
               // -> lpf
    1: m2_adder[M2RigidBodyMotions] -> {servos::GmtM2Hex}
    1: {servos::GmtFem}[M2PositionerNodes]

    // M1 edge sensor to RBMs feedback loop
    1: {servos::GmtFem}[M1EdgeSensors]!
        -> m1_es_to_rbm_int[Right<M1RigidBodyMotions>]
            -> adder

    // FEM state transfer to optical model
    // 1: {servos::GmtFem}[M1RigidBodyMotions] -> {agws::AgwsSh48}
    1: {servos::GmtFem}[M2State] -> {agws::AgwsSh48}
    // 1: {servos::GmtFem}[M1RigidBodyMotions] -> {agws::AgwsSh24}
    1: {servos::GmtFem}[M2State] -> {agws::AgwsSh24}
    // 1: {servos::GmtFem}[M1RigidBodyMotions] -> on_axis
    1: {servos::GmtFem}[M2State] -> on_axis
    // 1: {servos::GmtFem}[M1State] -> s1
    // 1000: s1[M1State] -> agws_wss
    1: {servos::GmtFem}[M2State] -> s2
    1000: s2[M2State] -> agws_wss[Wavefront] -> agws_wavefronts

    1: {servos::GmtFem}[M1RigidBodyMotions] -> m1_lom
    1: {servos::GmtFem}[M2RigidBodyMotions] -> m2_lom
    1: m1_lom[M1SegmentPiston].. -> m1_scopes
    1: m2_lom[M2SegmentPiston].. -> m2_scopes
    1: m1_lom[M1SegmentTipTilt].. -> m1_scopes
    1: m2_lom[M2SegmentTipTilt].. -> m2_scopes

    // // AGWS SH24 to FSMS feedback loop
    5: {agws::AgwsSh24Kernel}[M2FSMFsmCommand] -> fsm_pzt_int
    1: fsm_pzt_int[M2FSMFsmCommand] -> {servos::GmtM2}

    5000: {agws::AgwsSh48Kernel}[SensorData] -> sh48_m2_rbm_m1_bm_recon
    5000: sh48_m2_rbm_m1_bm_recon[SplitEstimate<0>]${42} -> pzt_to_rbm_int
        // -> m2_rbm_adder
    5000: sh48_m2_rbm_m1_bm_recon[SplitEstimate<1>]${27*7}
        -> sh48_int[Right<Estimate>] -> m1_bm_adder
    // 1000: {agws::AgwsSh48Kernel}[SensorData] -> mount_recon[MountEstimate] -> print
    // // 1000: {agws::AgwsSh48Kernel}[SensorData] -> pol//m1_recon//[Estimate] -> print
    // 1000: pzt_to_rbm[M2RigidBodyMotions]
    // //          -> pol[PseudoSensorData] -> mount_recon[Estimate]->print

    // 1: 2_lom[M2SegmentPiston].. -> m2_scopes
    // 1: m1_lomM1SegmentTipTilt].. -> m1_scopes
    // 1: m2_lom[M2Sclosed_loop_recon_sh48-to-m1-bm    // // AGWS SH24 to FSMS feedback loop

    // 1000: {agws::AgwsSh48Kernel}[SensorData] -> mount_recon[MountEstimate] -> print
    //          // -> pol[PseudoSensorData] -> sh48_m1_rbm_recon[Estimate]${42}->print
    // 100000: {agws::AgwsSh48}[Frame<Host>]! -> sh48_frame
    // 1000: {agws::AgwsSh24}[Frame<Host>]! -> sh24_frame

    1: on_axis[WfeRms<-9>].. -> shub
    1: on_axis[SegmentWfeRms<-9>].. -> shub
    1: on_axis[SegmentPiston<-9>].. -> shub
    1: on_axis[Mas<TipTilt>].. -> shub
    1: on_axis[Mas<SegmentTipTilt>].. -> shub
    // // 1: on_axis[SegmentPiston<-9>] -> scope_segment_piston
    1000: on_axis[Wavefront].. -> on_axis_wavefront
    }

    shub.lock().await.close().await?;
    (&mut *mount_scopes.lock().await).await?;
    m1_scopes.lock().await.close().await?;
    m2_scopes.lock().await.close().await?;

    Ok(())
}
