use std::{
    env,
    fs::{self, File},
    path::Path,
    time::Instant,
};

use faer::{Mat, MatRef};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{
    gain::Gain, integrator::Integrator, leftright, low_pass_filter::LowPassFilter,
    operator::Operator, timer::Timer,
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
#[cfg(feature = "qp")]
use gmt_dos_systems_agws::qp::{ActiveOptics, Estimate2OpticsState, QP};
use gmt_dos_systems_agws::{
    Agws,
    agws::{
        sh24::{Sh24, kernel::Sh24Kern},
        sh48::{Sh48, kernel::Sh48Kern},
    },
    builder::shack_hartmann::ShackHartmannBuilder,
};
use gmt_dos_systems_m1::SingularModes;
use gmt_fem::FEM;
#[cfg(not(feature = "qp"))]
use gmt_ns_im::agws::Sh48MergerReconstructor;
use gmt_ns_im::agws::TXY_RESIDUAL_SCALING;
use interface::{Left, Right, Tick};
use matio_rs::MatFile;

// const N_MODE: usize = 271;
// const M1_BM: usize = 27;
// const M1_RBM: usize = 41;
// const M2_RBM: usize = 41;

#[cfg(not(feature = "qp"))]
type K48 = Sh48MergerReconstructor<{ config::agws::sh48::RATE }>;
#[cfg(feature = "qp")]
type K48 = ActiveOptics<{ config::agws::sh48::RATE }, 41, 41, 27, 271>;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    let data_repo = Path::new(&env::var("DATA_REPO")?).join("main");
    fs::create_dir_all(&data_repo)?;
    unsafe {
        env::set_var("DATA_REPO", data_repo);
    }

    println!("FEM  : {}", env!("FEM_REPO"));
    println!("MOUNT: {}", env!("MOUNT_MODEL"));

    let now = Instant::now();

    let sim_sampling_frequency = 1000;
    let sim_duration = 60_usize; // second
    let bootstrapping_duration = 4_usize; // second
    let n_bootstrapping = sim_sampling_frequency * bootstrapping_duration;
    let n_sim = sim_sampling_frequency * sim_duration + 1;

    let mut fem = FEM::from_env()?;
    // println!("{fem}");

    let cfd_loads = Sys::<SigmoidCfdLoads>::try_from(
        CfdLoads::foh(".", sim_sampling_frequency)
            .duration((sim_duration + bootstrapping_duration) as f64)
            .windloads(&mut fem, Default::default()),
    )?;

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

        println!("Modes to forces matrices:");
        let b2f: Vec<_> = m1_sms
            .mode2force()
            .into_iter()
            .map(|mat| mat.columns(0, config::m1::segment::N_MODE).clone_owned())
            .inspect(|x| println!("{:?}", x.shape()))
            .collect();
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
            sim_sampling_frequency as f64,
            fem,
        )
        .wind_loads(WindLoads::new())
        .m1_segment_figure(M1SegmentFigure::new().transforms(s2b).modes_to_forces(b2f))
        .build()?
    };
    println!("{servos}");

    // AGWS
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh24/recon_sh24-to-pzt_pth.pkl")?,
        Default::default(),
    )?;
    println!("SH24 to FSM reconstructor:\n{recon}");

    #[cfg(feature = "qp")]
    let mut aco = {
        QP::<M1_RBM, M2_RBM, 27, N_MODE>::new(
            //"../aco_impl_stdalone/SHAcO_qp_rhoP1e-3_kIp5.rs.pkl")
            //"rustCalib_AcO_rhoP1e-12_kIp5.rs.pkl")
            Path::new("/home/ubuntu/projects/im-sim-scripts/aco_loop_example/data")
                .join("rustCalib_AcO_rhoP1e-12_kIp5.agws.pickle"),
        )?
        .update_calib(
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("qp")
                .join("sh48_calibration.pkl"),
        )?
        .build()?;
        println!("{aco}");
        aco.set_controller_gain(0.5f64);
        aco
    };
    let gmtb = Gmt::builder().m1(
        config::m1::segment::RAW_MODES,
        config::m1::segment::N_RAW_MODE,
    );
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
            // .atmosphere(atm, sim_sampling_frequency as f64)
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
        .sh24_calibration(recon);
        #[cfg(not(feature = "qp"))]
        let agws = {
            // agws.sh48_calibration(sh48_calibration(gmtb.clone(), config::m1::segment::N_MODE)?)

            use gmt_ns_im::agws::calibration::Sh48Calibration;
            agws.sh48_calibration(
                Sh48Calibration::new()?
                    .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)?
                    .recon()?,
            )
        };
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

    println!("Model built in {}s", now.elapsed().as_secs());

    let mut timer: Timer = Timer::new(n_bootstrapping);
    timer.progress();

    // let state_print = gmt_dos_clients::print::Print::new(8);

    let address = "127.0.0.1";
    let mut gmt_state_mon = Monitor::new();
    let gmt_state_tx = Transceiver::<OpticsState>::transmitter(address)?.run(&mut gmt_state_mon);
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

    // M2 RBM SH48 calibration
    let sh48_m2_rbm_recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh48/open_loop_recon_sh48-to-m2-rbm.pkl")?,
        Default::default(),
    )?;
    println!("SH48 to M2 RBM reconstructor:\n{sh48_m2_rbm_recon}");
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

    let sh48_m2_rbm_int = Integrator::new(42).gain(config::agws::sh48::INTEGRATOR_GAIN);
    let sh48_m1_bm_int =
        Integrator::new(7 * config::m1::segment::N_MODE).gain(config::agws::sh48::INTEGRATOR_GAIN);

    let m2_pos_lpf = LowPassFilter::new(42, 0.0063);

    // let print = Print::<Vec<f64>>::new(8);
    let timer: Timer = Timer::new(n_sim);
    type AgwsSh48 = Sh48<{ config::agws::sh48::RATE }>;
    type AgwsSh24 = Sh24<{ config::agws::sh24::RATE }>;
    type AgwsSh24Kernel = Sh24Kern<Sh24<{ config::agws::sh24::RATE }>>;
    type AgwsSh48Kernel = Sh48Kern<K48>;
    // let one_to_1000 = Sampler::default();
    // let e2o = Estimate2OpticsState::new();

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

    let optical_state_arrow = OpticalStateArrow::<M1State, M2RigidBodyMotions>::builder()
        .build(config::m1::segment::N_RAW_MODE);

    let split = leftright::LeftRight::<Estimate, leftright::Split>::split_chunks_at(
        6 + config::m1::segment::N_MODE,
        6,
    );

    let add_m2_rbms = Operator::plus();

    let m2_txy_scaling = Gain::new(vec![TXY_RESIDUAL_SCALING as f64; 42]);

    let m1_state = MirrorState::default().zeros_modes(config::m1::segment::N_MODE);
    // .set_zero_point(
    //     MirrorState::default()
    //         .zeros_modes(config::m1::segment::N_MODE)
    //         .set_segment_state(
    //             1,
    //             SegmentState::modes(vec![0.; config::m1::segment::N_MODE]).set_mode(0, 1e-6),
    //         ),
    // );

    // dbg!(&optical_state);
    // let state_print = gmt_dos_clients::print::Print::default().scale(1e9_f64);
    actorscript! {
        // #[model(state=running)]
    #[labels(//on_axis = "GMT Optics & Atmosphere\nw/ On-Axis Star",
        timer="⏲",
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

    1:  {servos::GmtFem}[OpticsState]! -> optical_state[OpticsState] -> {agws::AgwsSh24}
    1:   optical_state[OpticsState] -> {agws::AgwsSh48}

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
    5: {agws::AgwsSh24Kernel}[M2FSMFsmCommand] -> fsm_pzt_int
    1: fsm_pzt_int[M2FSMFsmCommand] -> {servos::GmtM2}

    // AGWS SH48 to M2 Txy and M1 bending modes loop
    1000: {agws::AgwsSh48Kernel}[Estimate]
        -> split[Left<Estimate>]
            -> m2_txy_scaling[Left<Estimate>]
                -> sh48_m2_rbm_int
    1000: split[Right<Estimate>]
        -> sh48_m1_bm_int[M1ModeShapes] -> m1_state
    1:  sh48_m2_rbm_int[Left<Estimate>]
                -> add_m2_rbms

    1: m1_state[M1State] -> {servos::GmtM1}

    }

    gmt_state_mon.drop(gmt_state_tx).await?;
    Ok(())
}
