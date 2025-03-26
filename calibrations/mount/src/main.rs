use std::{fs::File, path::Path};

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{integrator::Integrator, signals::Signals};
use gmt_dos_clients_arrow::Arrow;
use gmt_dos_clients_crseo::{
    OpticalModel,
    calibration::{Calib, CalibrationMode, Reconstructor},
};
use gmt_dos_clients_fem::{DiscreteModalSolver, solvers::ExponentialMatrix};
use gmt_dos_clients_io::{
    gmt_fem::{
        inputs::MCM2PZTF,
        outputs::{MCM2Lcl6D, MCM2PZTD, OSSM1Lcl},
    },
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::{
        M2RigidBodyMotions,
        fsm::{M2FSMFsmCommand, M2FSMPiezoForces, M2FSMPiezoNodes},
    },
    mount::{MountEncoders, MountSetPoint, MountTorques},
    optics::SensorData,
};
use gmt_dos_clients_mount::Mount;
use gmt_dos_systems_agws::{
    agws::{sh24::Sh24, sh48::Sh48},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::{Kernel, KernelFrame},
};
use gmt_dos_systems_m2::FSMS;
use gmt_ns_im::config;

async fn calibrate(channel: usize, path: impl AsRef<Path>) -> anyhow::Result<()> {
    let mount = Mount::new();

    let fsms = FSMS::<1>::new()?;

    let setpoint = Signals::new(3, 6000).channel(channel, 1e-6);
    let fem = DiscreteModalSolver::<ExponentialMatrix>::from_env()?
        .sampling(gmt_dos_clients_mount::sampling_frequency() as f64)
        .proportional_damping(2e-2)
        .ins::<MCM2PZTF>()
        .outs::<MCM2PZTD>()
        .including_mount()
        .outs::<OSSM1Lcl>()
        .outs::<MCM2Lcl6D>()
        .use_static_gain_compensation()
        .build()?;
    println!("{fem}");

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
    let int = Integrator::new(21).gain(0.5);

    // let gain = Gain::new(vec![1e6; 16]);
    // let print = Print::default();
    type Sh24Frame = KernelFrame<Sh24<{ config::agws::sh24::RATE }>>;
    type Sh48Frame = KernelFrame<Sh48<1000>>;
    actorscript!(
        #[model(name=mount_calibrations)]
        1: setpoint[MountSetPoint] -> mount[MountTorques] -> fem[MountEncoders]! -> mount
        1: {fsms}[M2FSMPiezoForces] -> fem[M2FSMPiezoNodes]! -> {fsms}
        1: fem[M1RigidBodyMotions] -> om24
        1: fem[M2RigidBodyMotions] -> om24
        1: fem[M1RigidBodyMotions] -> om48
        1: fem[M2RigidBodyMotions] -> om48
        5: om24[Sh24Frame]! -> k24[SensorData]
            -> recon[M2FSMFsmCommand] -> int[M2FSMFsmCommand]${21} -> {fsms}
        1000: om48[Sh48Frame]! -> k48[SensorData]${48*48*6}
        // 10: mount[MountTorques] -> prin*t
        // 10: fem[MountEncoders] -> gain[MountEncoders] -> print
    );
    mount_calibrations_logging_1000
        .lock()
        .await
        .to_parquet(path)?;
    Ok(())
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let path = Path::new("sh48_1murd_mount-az.parquet");
    if !path.exists() {
        println!(
            "Calibrating mount azimuth axis while closing the loop between the SH24 and the FSMS"
        );
        calibrate(0, path).await?;
    }
    let path = Path::new("sh48_1murd_mount-el.parquet");
    if !path.exists() {
        println!(
            "Calibrating mount elevation axis while closing the loop between the SH24 and the FSMS"
        );
        calibrate(1, path).await?;
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

    let mut mount_el = Arrow::from_parquet("sh48_1murd_mount-el.parquet")?;
    let mut el_data: Vec<f64> = mount_el.iter("SensorData")?.last().unwrap();
    println!("SH48 data size: {}", el_data.len());
    el_data.iter_mut().for_each(|x| *x *= 1e6);

    let mut mount_az = Arrow::from_parquet("sh48_1murd_mount-az.parquet")?;
    let mut az_data: Vec<f64> = mount_az.iter("SensorData")?.last().unwrap();
    println!("SH48 data size: {}", az_data.len());
    az_data.iter_mut().for_each(|x| *x *= 1e6);

    let data: Vec<_> = az_data
        .into_iter()
        .chain(el_data.into_iter())
        .zip(mask.iter().cycle())
        .filter_map(|(x, &m)| m.then_some(x))
        .collect();
    let calib = Calib::builder()
        .mode(CalibrationMode::Mount {
            elevation: 1e-6,
            azimuth: 1e-6,
        })
        .c(data)
        .mask(mask)
        .n_mode(2)
        .build();
    println!("{calib}");
    let mut recon = Reconstructor::from(calib);
    recon.pseudoinverse();
    println!("{recon}");
    let path = Path::new("recon_sh48-to-mount.pkl");
    println!(
        "Saving SH48 to mount reconstructor to {:?}",
        path.canonicalize()?
    );
    let mut file = File::create(path)?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    Ok(())
}
