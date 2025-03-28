use std::{fs::File, path::Path};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{
    integrator::Integrator,
    operator::{Left, Operator, Right},
    signals::Signals,
};
use gmt_dos_clients_arrow::Arrow;
use gmt_dos_clients_crseo::{
    OpticalModel,
    calibration::{Calib, CalibrationMode, Reconstructor},
};
use gmt_dos_clients_fem::{DiscreteModalSolver, DiscreteStateSpace, solvers::ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::{
        inputs::MCM2PZTF,
        outputs::{MCM2Lcl6D, MCM2PZTD, OSSM1EdgeSensors, OSSM1Lcl},
    },
    gmt_m1::{
        M1EdgeSensors, M1RigidBodyMotions,
        assembly::{M1ActuatorAppliedForces, M1HardpointsForces, M1HardpointsMotion},
    },
    gmt_m2::{
        M2RigidBodyMotions,
        fsm::{M2FSMFsmCommand, M2FSMPiezoForces, M2FSMPiezoNodes},
    },
    mount::{MountEncoders, MountTorques},
    optics::SensorData,
};
use gmt_dos_clients_mount::Mount;
use gmt_dos_systems_agws::{
    agws::{sh24::Sh24, sh48::Sh48},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::{Kernel, KernelFrame},
};
use gmt_dos_systems_m1::M1;
use gmt_dos_systems_m2::FSMS;
use gmt_ns_im::config;
use matio_rs::MatFile;

async fn calibrate(channel: usize, path: impl AsRef<Path>) -> anyhow::Result<()> {
    let mut fem = gmt_fem::FEM::from_env()?;

    let mount = Mount::new();

    let fsms = FSMS::<1>::new()?;

    let mut m1_rbm = vec![vec![0f64; 6]; 7];
    m1_rbm[6][channel] = 1e-6;
    let m1_rbm = Signals::from((m1_rbm, 6_000));
    let adder = Operator::new("+");
    let m1 = M1::<{ config::m1::segment::ACTUATOR_RATE }>::new(&mut fem)?;

    let m1_es_2_rbm: nalgebra::DMatrix<f64> =
        MatFile::load("../../m1-edge-sensors/es_2_rbm.mat")?.var("m1_r_es")?;
    let fem: DiscreteModalSolver<_> = DiscreteStateSpace::<ExponentialMatrix>::from(fem)
        .sampling(gmt_dos_clients_mount::sampling_frequency() as f64)
        .proportional_damping(2e-2)
        .ins::<MCM2PZTF>()
        .outs::<MCM2PZTD>()
        .including_mount()
        .including_m1(Some(vec![1, 2, 3, 4, 5, 6, 7]))?
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .outs_with::<OSSM1EdgeSensors>(m1_es_2_rbm.as_view())
        .use_static_gain_compensation()
        .build()?;
    // println!("{fem}");

    let sh48 = ShackHartmannBuilder::<1000>::sh48().use_calibration_src();
    let k48 = Kernel::<_>::try_from(&sh48)?;
    let om48 = OpticalModel::<_>::try_from(sh48)?;
    let sh24 = ShackHartmannBuilder::<{ config::agws::sh24::RATE }>::sh24().use_calibration_src();
    let k24 = Kernel::<_>::try_from(&sh24)?;
    let om24 = OpticalModel::<_>::try_from(sh24)?;

    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("../sh24/recon_sh24-to-pzt_pth.pkl")?,
        Default::default(),
    )?;
    let int = Integrator::new(21).gain(config::agws::sh24::INTEGRATOR_GAIN);
    let m1_es_to_rbm_int = Integrator::new(42).gain(config::m1::edge_sensor::RBM_INTEGRATOR_GAIN);

    // let gain = Gain::new(vec![1e6; 16]);
    // let print = Print::default();
    type Sh24Frame = KernelFrame<Sh24<{ config::agws::sh24::RATE }>>;
    type Sh48Frame = KernelFrame<Sh48<1000>>;
    actorscript!(
        #[model(name=m1_assembly_calibrations)]
        1: mount[MountTorques] -> fem[MountEncoders]! -> mount

        1: m1_rbm[Left<M1RigidBodyMotions>] -> adder[M1RigidBodyMotions]
            -> {m1}
        1: fem[M1EdgeSensors]!
            -> m1_es_to_rbm_int[Right<M1RigidBodyMotions>]
                -> adder

        1: fem[M1HardpointsMotion]! -> {m1}
        1: {m1}[M1HardpointsForces] -> fem
        1: {m1}[M1ActuatorAppliedForces] -> fem

        1: {fsms}[M2FSMPiezoForces] -> fem[M2FSMPiezoNodes]! -> {fsms}

        1: fem[M1RigidBodyMotions] -> om24
        1: fem[M2RigidBodyMotions] -> om24
        1: fem[M1RigidBodyMotions] -> om48
        1: fem[M2RigidBodyMotions] -> om48
        5: om24[Sh24Frame]! -> k24[SensorData]${24*24*2}
            -> recon[M2FSMFsmCommand]${21} -> int[M2FSMFsmCommand]! -> {fsms}
        1000: om48[Sh48Frame]! -> k48[SensorData]${48*48*6}
    );
    m1_assembly_calibrations_logging_1000
        .lock()
        .await
        .to_parquet(path)?;
    Ok(())
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let path = Path::new("sh48_1murd_m1-es-global-tip.parquet");
    if !path.exists() {
        println!(
            "Calibrating M1 assemply tip while closing the loops with the M1 edge sensors and between the SH24 and the FSMS"
        );
        calibrate(3, path).await?;
    }
    let path = Path::new("sh48_1murd_m1-es-global-tilt.parquet");
    if !path.exists() {
        println!(
            "Calibrating M1 assemply tilt while closing the loops with the M1 edge sensors and between the SH24 and the FSMS"
        );
        calibrate(4, path).await?;
    }

    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let k48 = Kernel::<Sh48<1>>::try_from(&sh48)?;
    let proc = k48.processor();
    let valids = proc.get_valid_lenslets();
    println!(
        "{:?} valid lenslets out of {}",
        proc.n_valid_lenslets(),
        valids.len()
    );
    let mask: Vec<bool> = valids.into_iter().map(|&x| x > 0).collect();

    let n = 48 * 48;
    let mask: Vec<_> = mask
        .chunks(n)
        .zip(mask.chunks(n))
        .flat_map(|(x, y)| {
            x.iter()
                .cloned()
                .chain(y.iter().cloned())
                .collect::<Vec<bool>>()
        })
        .collect();
    println!("mask: {}", mask.len());

    let mut m1_tip = Arrow::from_parquet("sh48_1murd_m1-es-global-tip.parquet")?;
    let mut tip_data: Vec<f64> = m1_tip.iter("SensorData")?.last().unwrap();
    println!("SH48 data size: {}", tip_data.len());
    tip_data.iter_mut().for_each(|x| *x *= 1e6);

    let mut m1_tilt = Arrow::from_parquet("sh48_1murd_m1-es-global-tilt.parquet")?;
    let mut tilt_data: Vec<f64> = m1_tilt.iter("SensorData")?.last().unwrap();
    println!("SH48 data size: {}", tilt_data.len());
    tilt_data.iter_mut().for_each(|x| *x *= 1e6);

    let data: Vec<_> = tip_data
        .into_iter()
        .chain(tilt_data.into_iter())
        .zip(mask.iter().cycle())
        .filter_map(|(x, &m)| m.then_some(x))
        .collect();
    let calib = Calib::builder()
        .mode(CalibrationMode::GlobalTipTilt(1e-6))
        .c(data)
        .mask(mask)
        .n_mode(2)
        .build();
    println!("{calib}");
    let mut recon = Reconstructor::from(calib);
    recon.pseudoinverse();
    println!("{recon}");
    let path = Path::new("recon_sh48-to-m1-assembly.pkl");
    println!("Saving SH48 to M1 assembly reconstructor to {:?}", path);
    let mut file = File::create(path)?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    Ok(())
}
