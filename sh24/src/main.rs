use std::fs::File;

use gmt_dos_actors::actorscript;
use gmt_dos_clients::{Tick, gain::Gain, timer::Timer};
use gmt_dos_clients_crseo::{OpticalModel, calibration::Reconstructor, sensors::Camera};
use gmt_dos_clients_io::gmt_m2::{M2RigidBodyMotions, fsm::M2FSMFsmCommand};
use gmt_dos_systems_agws::{
    Agws,
    agws::sh24::Sh24,
    builder::AgwsGuideStar,
    kernels::{Kernel, KernelFrame},
};

const SH48_RATE: usize = 1;
const SH24_RATE: usize = 1;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let sim_sampling_frequency = 1000;
    // let sim_duration = 1_usize; // second
    let n_step = 1; //sim_sampling_frequency * sim_duration;

    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("../calibrations/recon_sh24-to-rbm.pkl")?,
        Default::default(),
    )?;
    let agws = Agws::<SH48_RATE, SH24_RATE>::builder()
        // .source(AgwsGuideStar::sh24().fwhm(12.))
        // .load_atmosphere(
        //     "../atmosphere/atmosphere.toml",
        //     sim_sampling_frequency as f64,
        // )?
        .sh24_calibration(recon);
    let sh24 = agws.sh24().source(AgwsGuideStar::sh24().fwhm(12.));
    let sh24_kernel = Kernel::<Sh24<SH24_RATE>>::try_from(&sh24)?;
    let sh24 = OpticalModel::<Camera<SH24_RATE>>::try_from(sh24)?;
    let gain = Gain::new(vec![1e6; 42]);
    let timer: Timer = Timer::new(n_step);
    // let print = Print::default();
    //

    actorscript!(
        1: timer[Tick] -> sh24
        // 1000: sh24[Frame<Host>]!${n}
        1: sh24[KernelFrame<Sh24<SH24_RATE>>]!
                -> sh24_kernel[M2FSMFsmCommand]
                    -> gain[M2RigidBodyMotions]${42}
    );

    let log = &mut model_logging_1.lock().await;
    println!("{}", log);
    let data: Vec<Vec<f64>> = log.iter("M2RigidBodyMotions")?.collect();
    dbg!(data.len());
    dbg!(data.last());

    Ok(())
}
