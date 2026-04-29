use std::{error::Error, fs::File, time::Instant};

use gmt_dos_actors::{actorscript, client::Client};
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_crseo::{
    OpticalModel,
    calibration::Reconstructor,
    crseo::{self, FromBuilder, Gmt},
    sensors::NoSensor,
};
use gmt_dos_clients_io::{Estimate, gmt_m2::fsm::M2FSMFsmCommand, optics::WfeRms};
use gmt_dos_clients_optics_state::{OpticalState, OpticsState};
use gmt_dos_clients_servos::{GmtFem, GmtServoMechanisms};
use gmt_dos_systems_agws::{
    Agws,
    agws::{AgwsParts, sh24::Sh24},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::KernelFrame,
};
use gmt_ns_im::agws::{
    Sh48Reconstructor,
    calibration::{M2Txy, Sh48Calibration, Stack},
};
use interface::{Data, Read, Tick, UniqueIdentifier, Update, Write};

#[tokio::test(flavor = "multi_thread")]
async fn main() -> std::result::Result<(), Box<dyn Error>> {
    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
    let mut on_axis = OpticalModel::<NoSensor>::builder()
        .gmt(gmtb.clone())
        .build()?;

    let task0 = tokio::task::spawn(async move {
        <_ as Read<OpticsState>>::read(&mut on_axis, Data::new(OpticalState::default()));
        let now = Instant::now();
        loop {
            on_axis.update();
            if now.elapsed().as_secs() > 3 {
                break;
            }
        }
        <_ as Write<WfeRms<-9>>>::write(&mut on_axis)
            .map(|on_axis_wfe_rms| println!("On-axis WFE RMS: {:.0}nm", on_axis_wfe_rms[0]));
    });

    crseo::set_gpu(1);
    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
    let mut on_axis = OpticalModel::<NoSensor>::builder()
        .gmt(gmtb.clone())
        .build()?;

    let task1 = tokio::task::spawn(async move {
        crseo::set_gpu(1);
        <_ as Read<OpticsState>>::read(&mut on_axis, Data::new(OpticalState::default()));
        let now = Instant::now();
        loop {
            on_axis.update();
            if now.elapsed().as_secs() > 3 {
                break;
            }
        }
        <_ as Write<WfeRms<-9>>>::write(&mut on_axis)
            .map(|on_axis_wfe_rms| println!("On-axis WFE RMS: {:.0}nm", on_axis_wfe_rms[0]));
    });

    task0.await?;
    task1.await?;
    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn model() -> std::result::Result<(), Box<dyn Error>> {
    crseo::set_gpu(1);
    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
    let on_axis = OpticalModel::<NoSensor>::builder()
        .gmt(gmtb.clone())
        .build()?;
    let on_axis = Client::from(on_axis).cuda_device(1);

    crseo::set_gpu(0);
    let timer: Timer = Timer::new(1000);

    actorscript!(
        1: timer[Tick] -> on_axis
    );

    Ok(())
}

#[tokio::test(flavor = "multi_thread")]
async fn agws() -> std::result::Result<(), Box<dyn Error>> {
    env_logger::init();

    crseo::set_gpu(1);

    let sh24_recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh24/recon_sh24-to-pzt_pth.pkl")?,
        Default::default(),
    )?;
    let recon = Sh48Calibration::<Stack, M2Txy>::new()?
        .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)?
        .recon()?;
    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
    type K48 = Sh48Reconstructor<1>;
    type K24 = Sh24<5>;
    let AgwsParts {
        sh48,
        sh24,
        sh24_kernel,
        sh48_kernel,
        ..
    } = Agws::<1, 5, K48, K24>::builder()
        .gmt(gmtb.clone())
        .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
        .sh24_calibration(sh24_recon)
        .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
        .sh48_calibration(recon)
        .parts()?;
    let sh48 = Client::from(sh48).cuda_device(1);
    let sh24 = Client::from(sh24).cuda_device(1);
    let sh48_kernel = Client::from(sh48_kernel).cuda_device(1);
    let sh24_kernel = Client::from(sh24_kernel).cuda_device(1);

    crseo::set_gpu(0);

    let fem = gmt_fem::FEM::from_env()?;
    let servos =
        GmtServoMechanisms::<{ config::m1::segment::ACTUATOR_RATE }, 1>::new(1000_f64, fem)
            .build()?;

    pub struct Void;
    impl Update for Void {}
    impl<U: UniqueIdentifier> Read<U> for Void {
        fn read(&mut self, _: interface::Data<U>) {}
    }
    let void = Void;

    type AgwsSh24Frame = KernelFrame<K24>;
    type AgwsSh48Frame = KernelFrame<K48>;
    let timer: Timer = Timer::new(1000);
    actorscript!(
    1: timer[Tick] -> {servos::GmtFem}//[OpticsState] -> sh24
    1: timer[Tick] ->  sh24
    5: sh24[AgwsSh24Frame]! -> sh24_kernel[M2FSMFsmCommand] -> void
    // 1: timer[Tick] -> sh48[AgwsSh48Frame]! -> sh48_kernel[Estimate] -> void
    );
    Ok(())
}
