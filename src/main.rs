use std::{
    env,
    fs::{self, File},
    path::Path,
};

use faer::{Mat, MatRef};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    gain::Gain,
    integrator::{Integrator, Offset},
    leftright,
    low_pass_filter::LowPassFilter,
    operator::Operator,
    print::Print,
    select::Select,
    timer::Timer,
};
use gmt_dos_clients_crseo::{
    calibration::Reconstructor,
    crseo::{FromBuilder, Gmt},
};
// use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::Exponential};
use gmt_dos_clients_io::{
    Estimate,
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1ModeShapes,
    gmt_m2::{
        M2RigidBodyMotions,
        fsm::{M2FSMFsmCommand, M2FSMPiezoNodes},
    },
};
use gmt_dos_clients_servos::{
    GmtFem, GmtM1, GmtM2, GmtM2Hex, GmtServoMechanisms, M1SegmentFigure, WindLoads,
};

use gmt_dos_clients_optics_state::{
    M1State, MirrorState, OpticalState, OpticsState, SegmentState, arrow::OpticalStateArrow,
};
use gmt_dos_clients_transceiver::{Monitor, Transceiver};
use gmt_dos_clients_windloads::{
    CfdLoads,
    system::{M1, M2, Mount, SigmoidCfdLoads},
};
use gmt_dos_systems_agws::{
    Agws,
    agws::{AgwsParts, sh24::Sh24},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::KernelFrame,
};
use gmt_dos_systems_m1::SingularModes;
use gmt_fem::FEM;
use gmt_ns_im::{
    M2Txy, M2TxyToRxy,
    agws::{Sh48MergerReconstructor, TXY_RESIDUAL_SCALING, calibration::Sh48Calibration},
};
use interface::{Left, Right, Tick};
use matio_rs::MatFile;

type K48 = Sh48MergerReconstructor<{ config::agws::sh48::RATE }>;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    dotenvy::dotenv()?;

    let data_repo = Path::new(&env::var("DATA_REPO")?).join("main");
    fs::create_dir_all(&data_repo)?;
    unsafe {
        env::set_var("DATA_REPO", data_repo);
    }

    println!("FEM  : {}", env!("FEM_REPO"));
    println!("MOUNT: {}", env!("MOUNT_MODEL"));

    // let now = Instant::now();

    let mut fem = FEM::from_env()?;
    // println!("{fem}");

    // ===============================
    // -- CFD WINDLOADS --
    //
    // CFD wind loads are loaded either from a on-disk data file
    // or from S3 (credentials environment variables required)
    //
    // let store = object_store::local::LocalFileSystem::new();
    let store = object_store::aws::AmazonS3Builder::from_env()
        .with_region("us-east-1")
        .with_bucket_name("gmto.cfd.2025")
        .build()?;
    let cfd_loads = Sys::<SigmoidCfdLoads>::try_from(
        CfdLoads::foh(
            &format!("CASES/{}", config::WINDLOADS),
            // "/home/ubuntu/data/home/ubuntu/projects/gmt-ns-im",
            config::SIM_SAMPLING_FREQUENCY,
        )
        .duration(config::SIM_DURATION as f64)
        .windloads(&mut fem, Default::default())
        .fetch_and_build(store)
        .await?,
    )?;
    // let cfd_loads = Sys::<SigmoidCfdLoads>::try_from(
    //     CfdLoads::foh(".", config::SIM_SAMPLING_FREQUENCY)
    //         .duration((config::SIM_DURATION + config::BOOTSTRAPPING_DURATION) as f64)
    //         .windloads(&mut fem, Default::default())
    //         .build()?,
    // )?;
    // ===============================

    // M1 EDGE SENSORS TO RIGID-BODY MOTIONS TRANSFORM
    // let m1_es_2_rbm: nalgebra::DMatrix<f64> =
    //     MatFile::load("calibrations/m1/edge-sensors/es_2_rbm.mat")?.var("m1_r_es")?;
    // let servos =
    //     Sys::<GmtServoMechanisms<{ config::m1::ACTUATOR_RATE }, 1>>::from_data_repo_or_else(
    //         "servos.bin",
    //         || {
    //             GmtServoMechanisms::<{ config::m1::ACTUATOR_RATE }, 1>::new(
    //                 SIM_SAMPLING_FREQUENCY as f64,
    //                 fem,
    //             )
    //             .m1_segment_figure(M1SegmentFigure::new())
    //         },
    //     )?;

    // ===============================
    // -- SERVO-MECHANISMS --
    let servos = {
        // M1 segment structural modes are derived from M1 FEM
        // They are computing with the crate [gmt_dos-systems_m1-modes](https://github.com/rconan/dos-actors/tree/gmt-ns-im/systems/m1/modes)
        // see also: calibrations/m1/modes/README.md
        let m1_sms: SingularModes = serde_pickle::from_reader(
            // &File::open("calibrations/m1/modes/m1_singular_modes.pkl")?,
            &File::open(Path::new(env!("FEM_REPO")).join("m1_singular_modes.pkl"))?,
            Default::default(),
        )?;
        // Bending modes coefficients to actuator forces conversion
        println!("Modes to forces matrices:");
        let b2f: Vec<_> = m1_sms
            .mode2force()
            .into_iter()
            .map(|mat| mat.columns(0, config::m1::segment::N_MODE).clone_owned())
            .inspect(|x| println!("{:?}", x.shape()))
            .collect();
        // Segment figure to raw modes (influence functions) conversion
        println!("Surfaces to raw modes matrices:");
        let s2b: Vec<_> = m1_sms
            .raw_modes_into_mat()
            .into_iter()
            .map(|x| {
                let ncols = x.ncols();
                if ncols < config::m1::segment::N_RAW_MODE {
                    x.insert_columns(ncols, config::m1::segment::N_RAW_MODE - ncols, 0f64)
                } else {
                    x
                }
            })
            .map(|x| x.transpose())
            .inspect(|x| println!("{:?}", x.shape()))
            .collect();

        GmtServoMechanisms::<{ config::m1::segment::ACTUATOR_RATE }, 1>::new(
            config::SIM_SAMPLING_FREQUENCY as f64,
            fem,
        )
        .wind_loads(WindLoads::new())
        .m1_segment_figure(M1SegmentFigure::new().transforms(s2b).modes_to_forces(b2f))
        .build()?
    };
    println!("{servos}");
    // ===============================

    // ===============================
    // -- AGWS --
    // SH24 M2 segment tip-tilt reconstructor
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh24/recon_sh24-to-pzt_pth.pkl")?,
        Default::default(),
    )?;
    println!("SH24 to FSM reconstructor:\n{recon}");

    let gmtb = Gmt::builder().m1(
        config::m1::segment::RAW_MODES,
        config::m1::segment::N_RAW_MODE,
    );
    let AgwsParts {
        sh48,
        sh24,
        sh24_kernel,
        sh48_kernel,
        ..
    } = if config::ATMOSPHERE {
        Agws::<
            { config::agws::sh48::RATE },
            { config::agws::sh24::RATE },
            K48,
            Sh24<{ config::agws::sh24::RATE }>,
        >::builder()
        .load_atmosphere(
            "atmosphere/atmosphere.toml",
            config::SIM_SAMPLING_FREQUENCY as f64,
        )?
        // .atmosphere(atm, SIM_SAMPLING_FREQUENCY as f64)
    } else {
        Agws::<
            { config::agws::sh48::RATE },
            { config::agws::sh24::RATE },
            K48,
            Sh24<{ config::agws::sh24::RATE }>,
        >::builder()
        .sh24(if config::agws::sh24::CALIBRATION_SRC {
            ShackHartmannBuilder::sh24() // .source(AgwsGuideStar::sh24().zenith_azimuth(vec![0f32], vec![0f32]))
                .use_calibration_src()
        } else {
            ShackHartmannBuilder::sh24()
        })
        .sh48(if config::agws::sh48::CALIBRATION_SRC {
            ShackHartmannBuilder::sh48() // .source(AgwsGuideStar::sh48().zenith_azimuth(vec![0f32], vec![0f32]))
                .use_calibration_src()
        } else {
            ShackHartmannBuilder::sh48()
        })
    }
    .gmt(gmtb.clone())
    .sh24_calibration(recon)
    .sh48_calibration(
        // SH48 M2 (closed-loop) Txy and M1 bending modes reconstructor
        Sh48Calibration::new()?
            .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)?
            .recon()?,
    )
    .parts()?;
    // if let Some(p24) = config::agws::sh24::POINTING_ERROR {
    //     let _ = agws.sh24_pointing(p24).await;
    // }
    // println!("{agws}");
    // ===============================

    // M1 edge sensors to RBMs integrator
    // let m1_es_to_rbm_int = Integrator::new(42).gain(config::m1::edge_sensor::RBM_INTEGRATOR_GAIN);

    // println!("Model built in {}s", now.elapsed().as_secs());

    // ===============================
    // -- OPTICS STATE TRANSMITTER
    //
    // Broadcast the RBMs and figures of M1 and M2 segments to
    // the receiver in the scoring main script
    let address = "127.0.0.1";
    let mut gmt_state_mon = Monitor::new();
    let gmt_state_tx = Transceiver::<OpticsState>::transmitter(address)?.run(&mut gmt_state_mon);
    // ===============================

    // ===============================
    // -- FSM OFF-LOAD TO POSITIONER --
    //
    // It uses P. Thomson method to convert segment tip-tilt into
    // piston, tip and tilt FSM actuactor commands
    let matfile = MatFile::load("calibrations/sh24/m2_pzt_r.mat")?;
    let pzt_to_rbm: Vec<Mat<f64>> = (0..7)
        .map(|i| {
            let var: Vec<f64> = matfile.var(format!("var{i}")).unwrap();
            let mat = MatRef::from_column_major_slice(&var, 6, 6);
            mat.to_owned()
        })
        .collect();
    let pzt_to_rbm = Gain::<f64>::new(pzt_to_rbm);
    // -- FSM OFF-LOAD INTEGRATOR --
    let pzt_to_rbm_int = Integrator::new(42).gain(config::fsm::OFFLOAD_INTEGRATOR_GAIN);
    // -- FSM COMMAND INTEGRATOR --
    let fsm_pzt_int = Integrator::new(21).gain(config::agws::sh24::INTEGRATOR_GAIN);
    // ===============================

    // ===============================
    // -- SH48 M2 RBM integrator
    let sh48_m2_rbm_int = Integrator::new(42).gain(config::agws::sh48::INTEGRATOR_GAIN);
    // -- SH48 M1 bending modes integrator
    let sh48_m1_bm_int =
        Integrator::new(7 * config::m1::segment::N_MODE).gain(config::agws::sh48::INTEGRATOR_GAIN);
    // ===============================

    // ===============================
    // -- M2 POSITIONNER LOW-PASS FILTER
    //
    // Filter cut-off frequency sets to 1Hz
    let m2_pos_lpf =
        LowPassFilter::from_corner_frequency(42, 1f64, config::SIM_SAMPLING_FREQUENCY as f64);
    println!("{m2_pos_lpf}");
    let m2_pzt_lpf = LowPassFilter::from_corner_frequency(
        42,
        1f64,
        config::SIM_SAMPLING_FREQUENCY as f64 / 1000f64,
    );
    println!("{m2_pzt_lpf}");
    // ===============================

    let n_bootstrapping = config::SIM_SAMPLING_FREQUENCY * config::BOOTSTRAPPING_DURATION;
    let mut timer: Timer = Timer::new(n_bootstrapping);
    timer.progress();

    actorscript! {
        #[model(name=bootstrap)]
        #[labels(timer="⏲",
                 gmt_state_tx="🔊")]
    1: timer[Tick] -> {servos::GmtFem}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {servos::GmtFem}

    1: {servos::GmtFem}[OpticsState].. -> gmt_state_tx

    }

    // let print = Print::<Vec<f64>>::new(8);
    // type AgwsSh48 = Sh48<{ config::agws::sh48::RATE }>;
    type AgwsSh24 = Sh24<{ config::agws::sh24::RATE }>;
    // type AgwsSh24Kernel = Sh24Kern<AgwsSh24>;
    // type AgwsSh48Kernel = Sh48Kern<K48>;
    type AgwsSh24Frame = KernelFrame<AgwsSh24>;
    type AgwsSh48Frame = KernelFrame<K48>;
    // let one_to_1000 = Sampler::default();
    // let e2o = Estimate2OpticsState::new();

    // ===============================
    // -- GMT M1 AND M2 STATES --
    let mirror = if config::m1::POLISH_ERROR_MAPS == 0 {
        MirrorState::default()
    } else {
        MirrorState::from(
            SegmentState::modes(vec![0f64; config::m1::segment::N_RAW_MODE])
                .set_mode(config::m1::segment::N_RAW_MODE - 1, 1f64),
        )
    };
    let optical_state =
        OpticalState::m1(MirrorState::default().zeros_modes(config::m1::segment::N_RAW_MODE))
            .set_zero_point(OpticalState::m1(mirror));
    // -- STATES LOG --
    let optical_state_arrow = OpticalStateArrow::<M1State, M2RigidBodyMotions>::builder()
        .build(config::m1::segment::N_RAW_MODE - config::m1::POLISH_ERROR_MAPS);
    // ===============================

    // ===============================
    // -- SH48 ESTIMATE SPLITTER --
    //
    // Split the SH48 command vector between M2 RBMS and M1 bending modes coefficients
    // The M2 RBMs are all zeros expect for Tx and Ty
    // The command vector `c` is arranged segment wise i.e `c=[c1,c2,c3,c4,c5,c6,c7]`
    // and each `ci` is the concantenation of the 6 M2 segment RBMS and the M1 bending modes
    let split = leftright::LeftRight::<Estimate, leftright::Split>::split_chunks_at(
        6 + config::m1::segment::N_MODE,
        6,
    );
    // ===============================

    // ===============================
    // -- M2 SH48 RBMS (TXY) and SH24 (RXY) adder --
    let add_m2_rbms = Operator::plus();
    // ===============================

    // ===============================
    // -- M1 TXY RBM SCALING FACTOR --
    //
    // This is also the same scaling factor applied to the closed-loop calibration
    // matrix of M2 Txy
    let m2_txy_scaling = Gain::new(vec![TXY_RESIDUAL_SCALING as f64; 42]);
    // ===============================

    // ===============================
    // -- GMT M1 STATE --
    let m1_state = MirrorState::default().zeros_modes(config::m1::segment::N_MODE);
    // .set_zero_point(
    //     MirrorState::default()
    //         .zeros_modes(config::m1::segment::N_MODE)
    //         .set_segment_state(
    //             1,
    //             SegmentState::modes(vec![0.; config::m1::segment::N_MODE]).set_mode(0, 1e-6),
    //         ),
    // );
    // ===============================

    // ===============================
    // -- FAST SEGMENT TIP-TILT --
    let n_sim = config::SIM_SAMPLING_FREQUENCY * config::FAST_SEGMENT_TIPTILT_DURATION;
    let timer: Timer = Timer::new(n_sim);
    actorscript! {
    #[model(name=fast_segment_tip_tilt)]
    #[labels(//on_axis = "GMT Optics & Atmosphere\nw/ On-Axis Star",
        timer="⏲",
         fsm_pzt_int="FSM\nIntegrator",
         // pzt_to_rbm="FSM\nto\nPositioner",
         // pzt_to_rbm_int="Positioner\nIntegrator",
         // split="Split SH48 Estimate into\nM2RigidBodyMotions(Left)\n& M1ModeShapes(Right)",
         // add_m2_rbms="+",
         sh24_kernel="SH24 Kernel",
         optical_state_arrow="Optics State\nLog",
         // sh48_m2_rbm_int = "∫ M2RigidBodyMotions",
         // sh48_m1_bm_int = "∫ M1ModeShapes",
         gmt_state_tx="🔊"
         )]
    1: timer[Tick] -> {servos::GmtFem}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {servos::GmtFem}

    1: optical_state[OpticsState]!.. -> gmt_state_tx
    1: optical_state[OpticsState]!.. -> optical_state_arrow


    // FSM to positionner off-load
    // 1: {servos::GmtFem}[M2PositionerNodes]
    // 1: {servos::GmtFem}[M2FSMPiezoNodes]
    //     -> pzt_to_rbm[M2RigidBodyMotions] //-> scope_fsm_cmd
    //         -> m2_pos_lpf[M2RigidBodyMotions]
    //             -> pzt_to_rbm_int[Right<M2RigidBodyMotions>]
    //                 -> add_m2_rbms[M2RigidBodyMotions]
    //                     -> {servos::GmtM2Hex}

    // M1 edge sensor to RBMs feedback loop
    // 1: {servos::GmtFem}[M1EdgeSensors]!
    //     -> m1_es_to_rbm_int[M1RigidBodyMotions]
    //         -> {servos::GmtM1}
            // -> adder


    // AGWS SH24 to FSMS feedback loop
    1:  {servos::GmtFem}[OpticsState]! -> optical_state[OpticsState] ->  sh24
    5: sh24[AgwsSh24Frame]! -> sh24_kernel[M2FSMFsmCommand] -> fsm_pzt_int
    1: fsm_pzt_int[M2FSMFsmCommand] -> {servos::GmtM2}
    }
    // ===============================

    // ===============================
    // -- HIGH GAIN ADAPTIVE OPTICS --

    // let aprint = Print::new(6);
    let m2_txy_2_rxy = M2TxyToRxy::new()?;

    let n_sim = config::SIM_SAMPLING_FREQUENCY * config::HIGH_GAIN_ACO_DURATION + 1;
    let timer: Timer = Timer::new(n_sim);
    actorscript! {
    #[labels(//on_axis = "GMT Optics & Atmosphere\nw/ On-Axis Star",
        timer="⏲",
         sh24_kernel="SH24 Kernel",
         sh48_kernel="SH48 Kernel",
         fsm_pzt_int="FSM\nIntegrator",
         pzt_to_rbm="FSM\nto\nPositioner",
         pzt_to_rbm_int="Positioner\nIntegrator",
         split="Split SH48 Estimate into\nM2RigidBodyMotions(Left)\n& M1ModeShapes(Right)",
         add_m2_rbms="+",
         optical_state_arrow="Optics State\nLog",
         sh48_m2_rbm_int = "∫ M2RigidBodyMotions",
         sh48_m1_bm_int = "∫ M1ModeShapes",
         gmt_state_tx="🔊"
         )]
    1: timer[Tick] -> {servos::GmtFem}

    1: {cfd_loads::M1}[CFDM1WindLoads] -> {servos::GmtFem}
    1: {cfd_loads::M2}[CFDM2WindLoads] -> {servos::GmtFem}
    1: {cfd_loads::Mount}[CFDMountWindLoads] -> {servos::GmtFem}


    1: optical_state[OpticsState]!.. -> gmt_state_tx
    1: optical_state[OpticsState]!.. -> optical_state_arrow


    // FSM to positionner off-load
    1: {servos::GmtFem}[M2PositionerNodes]
    1: {servos::GmtFem}[M2FSMPiezoNodes]
        -> pzt_to_rbm[M2RigidBodyMotions] //-> scope_fsm_cmd
            -> m2_pos_lpf[M2RigidBodyMotions]
                -> pzt_to_rbm_int[Right<M2RigidBodyMotions>]
                    -> add_m2_rbms[M2RigidBodyMotions]
                        -> {servos::GmtM2Hex}

    // M1 edge sensor to RBMs feedback loop
    // 1: {servos::GmtFem}[M1EdgeSensors]!
    //     -> m1_es_to_rbm_int[M1RigidBodyMotions]
    //         -> {servos::GmtM1}
            // -> adder


    // AGWS SH24 to FSMS feedback loop
    1:  {servos::GmtFem}[OpticsState]! -> optical_state[OpticsState] -> sh24
    5: sh24[AgwsSh24Frame]! -> sh24_kernel[M2FSMFsmCommand] -> fsm_pzt_int
    1: fsm_pzt_int[M2FSMFsmCommand] -> {servos::GmtM2}

    // AGWS SH48 to M2 Txy and M1 bending modes loop
    1:   optical_state[OpticsState] -> sh48
    1000: sh48[AgwsSh48Frame]! -> sh48_kernel[Estimate]
        -> split[Left<Estimate>]
            -> m2_txy_scaling[Left<Estimate>]
                -> sh48_m2_rbm_int
    1000: split[Right<Estimate>]
        -> sh48_m1_bm_int[M1ModeShapes] -> m1_state
    1000:  sh48_m2_rbm_int[Left<Estimate>]
            -> m2_txy_2_rxy[Left<Estimate>]
                -> add_m2_rbms
    1: m1_state[M1State] -> {servos::GmtM1}

    // 1000:  sh48_m2_rbm_int[Left<Estimate>] -> m2_pzt_lpf[Left<Estimate>] -> m2_txy_2_rxy
    // 5: m2_txy_2_rxy[Offset<M2FSMFsmCommand>] -> fsm_pzt_int
    }
    // ===============================

    gmt_state_mon.drop(gmt_state_tx).await?;
    Ok(())
}
