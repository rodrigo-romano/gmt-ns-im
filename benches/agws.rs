use std::{fs::File, path::Path};

use criterion::*;
use gmt_dos_actors::{
    actor::{Actor, Terminator}, client::Client, model::Model, prelude::{AddActorOutput, AddOuput, TryIntoInputs, vec_box}, system::Sys
};
use gmt_dos_clients::timer::Timer;
use gmt_dos_clients_crseo::calibration::Reconstructor;
use gmt_dos_clients_io::{Estimate, gmt_m2::fsm::M2FSMFsmCommand};
use gmt_dos_systems_agws::{
    Agws,
    agws::{
        AgwsParts,
        sh24::{Sh24, kernel::Sh24Kern},
        sh48::{Sh48, kernel::Sh48Kern},
    },
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::KernelFrame,
};
use gmt_ns_im::agws::{
    Sh48Reconstructor,
    calibration::{M2Txy, Stack},
};
use interface::{Read, Tick, UniqueIdentifier, Update};
// use qp::sh24::{Sh48MergerReconstructor, calibration::Sh48Calibration};

const SH48_RATE: usize = 5000;
const SH24_RATE: usize = 5;
type K48 = Sh48Reconstructor<SH48_RATE>;
type Sh48Calibration = gmt_ns_im::agws::calibration::Sh48Calibration<Stack, M2Txy>;

pub struct Void;
impl Update for Void {}
impl<U: UniqueIdentifier> Read<U> for Void {
    fn read(&mut self, _: interface::Data<U>) {}
}

// #[inline]
async fn model(
    // AgwsParts {
    //     sh48,
    //     sh24,
    //     sh24_kernel,
    //     sh48_kernel,
    // }: AgwsParts<SH48_RATE, SH24_RATE, K48, Sh24<SH24_RATE>>,
    mut sh48: Actor<Sh48<SH48_RATE>, 1, SH48_RATE>,
    mut sh24: Actor<Sh24<SH24_RATE>, 1, SH24_RATE>,
    mut sh48_kernel: Actor<Sh48Kern<Sh48Reconstructor<SH48_RATE>>, SH48_RATE, SH48_RATE>,
    mut sh24_kernel: Actor<Sh24Kern<Sh24<SH24_RATE>>, SH24_RATE, SH24_RATE>,
) {
    // let mut sh48: Actor<_, 1, SH48_RATE> = sh48.into();
    // let mut sh48_kernel: Actor<_, SH48_RATE, SH48_RATE> = sh48_kernel.into();
    // let mut sh24: Actor<_, 1, SH24_RATE> = sh24.into();
    // let mut sh24_kernel: Actor<_, SH24_RATE, SH24_RATE> = sh24_kernel.into();
    // let mut agws = Sys::<Agws<SH48_RATE, SH24_RATE, K48, Sh24<SH24_RATE>>>::clone(agws_prime);
    let mut timer: Actor<Timer, 0, 1> = Timer::new(100).into();
    timer
        .add_output()
        .multiplex(2)
        .build::<Tick>()
        .into_input::<Sh48<SH48_RATE>>(&mut sh48)
        .into_input::<Sh24<SH24_RATE>>(&mut sh24)
        .unwrap();
    let mut void_48: Terminator<Void, SH48_RATE> = Void.into();
    sh48.add_output()
        .build::<KernelFrame<K48>>()
        .into_input(&mut sh48_kernel)
        .unwrap();
    sh48_kernel
        .add_output()
        .build::<Estimate>()
        .into_input(&mut void_48)
        .unwrap();
    let mut void_24: Terminator<Void, SH24_RATE> = Void.into();
    sh24.add_output()
        .build::<KernelFrame<Sh24<SH24_RATE>>>()
        .into_input(&mut sh24_kernel)
        .unwrap();
    sh24_kernel
        .add_output()
        .build::<M2FSMFsmCommand>()
        .into_input(&mut void_24)
        .unwrap();

    Model::new(vec_box!(
        timer,
        sh48,
        sh24,
        sh24_kernel,
        sh48_kernel,
        void_48,
        void_24
    ))
    .quiet()
    .check()
    .unwrap()
    .run()
    .await
    .unwrap();
}

pub fn agws(c: &mut Criterion) {
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open(
            Path::new("/home/ubuntu/projects/gmt-ns-im")
                .join("calibrations/sh24/recon_sh24-to-pzt_pth.pkl"),
        )
        .unwrap(),
        Default::default(),
    )
    .unwrap();

    let AgwsParts {
        sh48,
        sh24,
        sh24_kernel,
        sh48_kernel,
    }: AgwsParts<SH48_RATE, SH24_RATE, K48, Sh24<SH24_RATE>> =
        Agws::<SH48_RATE, SH24_RATE, K48, Sh24<SH24_RATE>>::builder()
            .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
            .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
            // .gmt(Gmt::builder().m1(
            //     config::m1::segment::RAW_MODES,
            //     config::m1::segment::N_RAW_MODE,
            // ))
            .sh24_calibration(recon)
            .sh48_calibration(
                Sh48Calibration::new()
                    .unwrap()
                    .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)
                    .unwrap()
                    .recon()
                    .unwrap(),
            )
            .parts()
            .unwrap();
    let sh48 = Client::from(sh48).blocking();
    let sh48: Actor<_, 1, SH48_RATE> = (&sh48).into();
    let sh48_kernel: Actor<_, SH48_RATE, SH48_RATE> = sh48_kernel.into();
    let sh24 = Client::from(sh24).blocking();
    let sh24: Actor<_, 1, SH24_RATE> = (&sh24).into();
    let sh24_kernel: Actor<_, SH24_RATE, SH24_RATE> = sh24_kernel.into();

    c.bench_function("AGWS", |b| {
        b.to_async(tokio::runtime::Runtime::new().unwrap())
            .iter(|| {
                model(
                    sh48.clone(),
                    sh24.clone(),
                    sh48_kernel.clone(),
                    sh24_kernel.clone(),
                )
            })
    });
}

pub fn agws_atm(c: &mut Criterion) {
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open(
            Path::new("/home/ubuntu/projects/gmt-ns-im")
                .join("calibrations/sh24/recon_sh24-to-pzt_pth.pkl"),
        )
        .unwrap(),
        Default::default(),
    )
    .unwrap();

    let agws = Agws::<SH48_RATE, SH24_RATE, K48, Sh24<SH24_RATE>>::builder()
        .sh24(ShackHartmannBuilder::sh24())
        .sh48(ShackHartmannBuilder::sh48())
        // .gmt(Gmt::builder().m1(
        //     config::m1::segment::RAW_MODES,
        //     config::m1::segment::N_RAW_MODE,
        // ))
        .sh24_calibration(recon)
        .sh48_calibration(
            Sh48Calibration::new()
                .unwrap()
                .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)
                .unwrap()
                .recon()
                .unwrap(),
        )
        .atmosphere(Default::default(), 1e3)
        .build()
        .unwrap();
    // let timer

    // c.bench_function("AGWS w/ Atmosphere", |b| {
    //     b.to_async(tokio::runtime::Runtime::new().unwrap())
    //         .iter(|| model(&agws))
    // });
}

pub fn agws_atm_ray_trace(c: &mut Criterion) {
    let recon: Reconstructor = serde_pickle::from_reader(
        File::open(
            Path::new("/home/ubuntu/projects/gmt-ns-im")
                .join("calibrations/sh24/recon_sh24-to-pzt_pth.pkl"),
        )
        .unwrap(),
        Default::default(),
    )
    .unwrap();

    let agws = Agws::<SH48_RATE, SH24_RATE, K48, Sh24<SH24_RATE>>::builder()
        .sh24(ShackHartmannBuilder::sh24())
        .sh48(ShackHartmannBuilder::sh48())
        // .gmt(Gmt::builder().m1(
        //     config::m1::segment::RAW_MODES,
        //     config::m1::segment::N_RAW_MODE,
        // ))
        .sh24_calibration(recon)
        .sh48_calibration(
            Sh48Calibration::new()
                .unwrap()
                .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)
                .unwrap()
                .recon()
                .unwrap(),
        )
        .load_atmosphere(
            "/home/ubuntu/projects/gmt-ns-im/atmosphere/atmosphere.toml",
            1e3,
        )
        .unwrap()
        .build()
        .unwrap();
    // let timer

    // c.bench_function("AGWS w/ Ray Trace Atmosphere", |b| {
    //     b.to_async(tokio::runtime::Runtime::new().unwrap())
    //         .iter(|| model(&agws))
    // });
}

criterion_group!(benches, agws, agws_atm, agws_atm_ray_trace);
criterion_main!(benches);
