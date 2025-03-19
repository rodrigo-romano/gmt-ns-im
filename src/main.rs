use std::{fs::File, io::BufReader, time::Instant};

use faer::{Mat, MatRef};
use gmt_dos_actors::{actorscript, system::Sys};
use gmt_dos_clients::{gain::Gain, gif, integrator::Integrator, signals::Signals, timer::Timer};
use gmt_dos_clients_crseo::{
    OpticalModel,
    calibration::Reconstructor,
    crseo::{FromBuilder, Gmt, builders::AtmosphereBuilder},
    sensors::NoSensor,
};
// use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::Exponential};
use gmt_dos_clients_io::{
    cfd_wind_loads::{CFDM1WindLoads, CFDM2WindLoads, CFDMountWindLoads},
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::{
        M2PositionerNodes, M2RigidBodyMotions,
        fsm::{M2FSMFsmCommand, M2FSMPiezoNodes},
    },
    mount::MountSetPoint,
    optics::{SegmentTipTilt, SegmentWfeRms, TipTilt, Wavefront, WfeRms},
};
use gmt_dos_clients_scope::server::{Monitor, Scope};
use gmt_dos_clients_servos::{
    GmtFem, GmtM2, GmtM2Hex, GmtMount, GmtServoMechanisms, M1SegmentFigure,
};
use gmt_dos_clients_windloads::{
    CfdLoads,
    system::{M1, M2, Mount, SigmoidCfdLoads},
};
use gmt_dos_systems_agws::{
    Agws,
    agws::{sh24::Sh24, sh48::Sh48},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::Kernel,
};
use gmt_fem::FEM;
use gmt_ns_im::config;
use interface::units::{Mas, MuM};
use interface::{Tick, filing::Filing};
use matio_rs::MatFile;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    env_logger::init();

    println!("FEM  : {}", env!("FEM_REPO"));
    println!("MOUNT: {}", env!("MOUNT_MODEL"));

    let now = Instant::now();

    let sim_sampling_frequency = 1000;
    let sim_duration = 10_usize; // second
    let bootstrapping_duration = 4_usize; // second
    let n_bootstrapping = sim_sampling_frequency * bootstrapping_duration;

    let mut fem = FEM::from_env()?;
    // println!("{fem}");

    let cfd_loads = Sys::<SigmoidCfdLoads>::try_from(
        CfdLoads::foh(".", sim_sampling_frequency)
            .duration((sim_duration + bootstrapping_duration) as f64)
            .mount(&mut fem, 0, None)
            .m1_segments()
            .m2_segments(),
    )?;

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
    let servos = GmtServoMechanisms::<{ config::m1::ACTUATOR_RATE }, 1>::new(
        sim_sampling_frequency as f64,
        fem,
    )
    // .m1_segment_figure(M1SegmentFigure::new())
    .build()?;
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
    let agws: Sys<Agws<{ config::agws::sh48::RATE }, { config::agws::sh24::RATE }>> =
        Agws::<{ config::agws::sh48::RATE }, { config::agws::sh24::RATE }>::builder()
            .gmt(Gmt::builder().m1(
                gmt_ns_im::config::m1::RAW_MODES,
                gmt_ns_im::config::m1::N_RAW_MODE,
            ))
            .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
            // .load_atmosphere("atmosphere/atmosphere.toml", sim_sampling_frequency as f64)?
            .sh24_calibration(recon)
            .build()?;
    println!("{agws}");

    // let sh48_frame: gif::Frame<f32> = gif::Frame::new("sh48_frame.png", 48 * 8);
    // let sh24_frame: gif::Frame<f32> = gif::Frame::new("sh24_frame.png", 24 * 12);
    // let on_axis_wavefront: gif::Frame<f64> = gif::Frame::new("on-axis_wavefront.png", 512);
    let on_axis_wavefront: gif::Gif<f64> = gif::Gif::new("on-axis_wavefront.gif", 512, 512)?;

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
    let pzt_to_rbm_int = Integrator::new(42).gain(1e-3);

    // On-axis scoring star
    // let atm = AtmosphereBuilder::load("atmosphere/atmosphere.toml")?;
    let on_axis = OpticalModel::<NoSensor>::builder()
        .gmt(Gmt::builder().m1(config::m1::RAW_MODES, config::m1::N_RAW_MODE))
        // .atmosphere(atm)
        .sampling_frequency(sim_sampling_frequency as f64)
        .build()?;

    println!("Model built in {}s", now.elapsed().as_secs());

    // SCOPES
    let mut monitor = Monitor::new();
    // let scope_segment_piston = Scope::<SegmentPiston<-9>>::builder(&mut monitor).build()?;
    let scope_segment_wfe_rms = Scope::<SegmentWfeRms<-9>>::builder(&mut monitor).build()?;
    let scope_wfe_rms = Scope::<WfeRms<-9>>::builder(&mut monitor).build()?;
    let scope_tiptilt = Scope::<Mas<TipTilt>>::builder(&mut monitor).build()?;
    let scope_segment_tiptilt = Scope::<Mas<SegmentTipTilt>>::builder(&mut monitor).build()?;
    // let scope_fsm_cmd = Scope::<M2FSMFsmCommand>::builder(&mut monitor).build()?;
    // let scope_m2_rbm = Scope::<MuM<M2RigidBodyMotions>>::builder(&mut monitor).build()?;
    // ---

    // let m2_rbm = Signals::new(42, 3000 + n_bootstrapping).channel(3, 1e-6);
    let mount_cmd = Signals::new(3, 6000 + n_bootstrapping).channel(1, 1e-6);
    // Bootstrapping the FEM and associated controls
    // let fem = state_space;
    let timer: Timer = Timer::new(n_bootstrapping);
    actorscript! {
        #[model(name=bootstrap)]
    1: timer[Tick] -> {servos::GmtFem}
    // 1: m2_rbm[M2RigidBodyMotions] -> {servos::GmtM2Hex}
    // 1: {servos::GmtFem}[MuM<M2RigidBodyMotions>]! -> scope_m2_rbm
    // 1: {cfd_loads::M1}[CFDM1WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::M2}[CFDM2WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::Mount}[CFDMountWindLoads] -> {servos::GmtFem}
    1: mount_cmd[MountSetPoint] -> {servos::GmtMount}

    1: {servos::GmtFem}[M1RigidBodyMotions] -> on_axis
    1: {servos::GmtFem}[M2RigidBodyMotions] -> on_axis

    1: on_axis[WfeRms<-9>]$.. -> scope_wfe_rms
    1: on_axis[SegmentWfeRms<-9>]$.. -> scope_segment_wfe_rms
    1: on_axis[Mas<TipTilt>]$.. -> scope_tiptilt
    1: on_axis[Mas<SegmentTipTilt>]$.. -> scope_segment_tiptilt
    // // 1: on_axis[SegmentPiston<-9>] -> scope_segment_piston
    1000: on_axis[Wavefront].. -> on_axis_wavefront
    }

    let timer: Timer = Timer::new(6000);
    type AgwsSh48 = Sh48<{ config::agws::sh48::RATE }>;
    type AgwsSh24 = Sh24<{ config::agws::sh24::RATE }>;
    type AgwsSh24Kernel = Kernel<Sh24<{ config::agws::sh24::RATE }>>;
    actorscript! {
        // #[model(state=running)]
    #[labels(on_axis = "GMT Optics & Atmosphere\nw/ On-Axis Star",
         fsm_pzt_int="Integrator")]//,
         // scope_wfe_rms="Scope", scope_segment_wfe_rms="Scope")]
         // sh48_frame = "SH48\nframe")]//, sh24_frame = "SH24\nframe")]
    // 1: timer[Tick] -> {servos::GmtFem}
    // 1: {cfd_loads::M1}[CFDM1WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::M2}[CFDM2WindLoads] -> {servos::GmtFem}
    // 1: {cfd_loads::Mount}[CFDMountWindLoads] -> {servos::GmtFem}
    //
    1: mount_cmd[MountSetPoint] -> {servos::GmtMount}

    // 1: m2_rbm[M2RigidBodyMotions] -> {servos::GmtM2Hex}
    // 1: {servos::GmtFem}[MuM<M2RigidBodyMotions>]! -> scope_m2_rbm

    1: {servos::GmtFem}[M2FSMPiezoNodes]${42}
        -> pzt_to_rbm[M2RigidBodyMotions] //-> scope_fsm_cmd
            -> pzt_to_rbm_int[M2RigidBodyMotions]
                -> {servos::GmtM2Hex}
    1: {servos::GmtFem}[M2PositionerNodes]${84}

    // FEM state transfer to optical model
    1: {servos::GmtFem}[M1RigidBodyMotions]! -> {agws::AgwsSh48}
    1: {servos::GmtFem}[M2RigidBodyMotions]! -> {agws::AgwsSh48}
    1: {servos::GmtFem}[M1RigidBodyMotions]! -> {agws::AgwsSh24}
    1: {servos::GmtFem}[M2RigidBodyMotions]! -> {agws::AgwsSh24}
    1: {servos::GmtFem}[M1RigidBodyMotions] -> on_axis
    1: {servos::GmtFem}[M2RigidBodyMotions] -> on_axis

    // // AGWS SH24 to FSMS feedback loop
    5: {agws::AgwsSh24Kernel}[M2FSMFsmCommand] -> fsm_pzt_int
    1: fsm_pzt_int[M2FSMFsmCommand] -> {servos::GmtM2}

    // 5000: {agws::AgwsSh48}[Frame<Host>]! -> sh48_frame
    // 50: {agws::AgwsSh24}[Frame<Host>]! -> sh24_frame

    1: on_axis[WfeRms<-9>]$.. -> scope_wfe_rms
    1: on_axis[SegmentWfeRms<-9>]$.. -> scope_segment_wfe_rms
    1: on_axis[Mas<TipTilt>]$.. -> scope_tiptilt
    1: on_axis[Mas<SegmentTipTilt>]$.. -> scope_segment_tiptilt
    // // 1: on_axis[SegmentPiston<-9>] -> scope_segment_piston
    1000: on_axis[Wavefront].. -> on_axis_wavefront
    }

    // model_logging_1000.lock().await.save();

    scope_wfe_rms.lock().await.end_transmission();
    scope_segment_wfe_rms.lock().await.end_transmission();
    scope_tiptilt.lock().await.end_transmission();
    scope_segment_tiptilt.lock().await.end_transmission();
    // scope_fsm_cmd.end_transmission();
    // scope_m2_rbm.lock().await.end_transmission();
    monitor.join().await?;

    Ok(())
}
