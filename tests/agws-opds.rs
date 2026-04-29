use std::{error::Error, fs::File, iter};

use gmt_dos_clients::gif::{self};
use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder,
    calibration::{MixedMirrorMode, Reconstructor},
    crseo::{FromBuilder, Gmt},
    sensors::{NoSensor, WaveSensor, builders::WaveSensorBuilder},
};
use gmt_dos_clients_io::{
    Estimate,
    gmt_m2::M2RigidBodyMotions,
    optics::{Wavefront, WfeRms},
};
use gmt_dos_clients_optics_state::{MirrorState, OpticalState, OpticsState, SegmentState};
use gmt_dos_systems_agws::{
    Agws,
    agws::{AgwsParts, sh24::Sh24TT},
    builder::shack_hartmann::{AgwsGuideStar, ShackHartmannBuilder},
    kernels::KernelFrame,
};
use gmt_ns_im::agws::{
    Sh48Reconstructor,
    calibration::{M2Txy, Sh48Calibration, Stack},
};
use interface::{Data, Read, TryRead, TryUpdate, TryWrite, Update, Write};

type K48 = Sh48Reconstructor<1>;
type K24 = Sh24TT<1>;

fn print_rbms<'a, const M_ID: u8>(
    data: impl Iterator<Item = &'a [f64]>,
    modes: Option<impl Iterator<Item = &'a [f64]>>,
) {
    if let Some(modes) = modes {
        println!("[M{M_ID} RBMS (x1e6)] M1 BMs RSS (x1e9)");
        data.map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
            .zip(
                modes
                    // .inspect(|y| {
                    //     dbg!(y);
                    // })
                    .map(|x| x.iter().map(|x| x * 1e9).map(|x| x * x).sum::<f64>())
                    .map(|x| x.sqrt()),
            )
            .enumerate()
            .for_each(|(i, (x, y))| println!(" {}:{x:+5.1?} {y:+6.1}", i + 1));
    } else {
        println!("[M{M_ID} RBMS (x1e6)]");
        data.map(|x| x.iter().map(|x| x * 1e6).collect::<Vec<_>>())
            .enumerate()
            .for_each(|(i, x)| println!(" {}:{x:+5.1?}", i + 1));
    }
}

type Sh48WaveSensor = OpticalModel<WaveSensor>;

fn agws(
    reconstructor: Reconstructor<MixedMirrorMode>,
) -> std::result::Result<(AgwsParts<1, 1, K48, K24>, Sh48WaveSensor), Box<dyn Error>> {
    let sh24_recon: Reconstructor = serde_pickle::from_reader(
        File::open("calibrations/sh24/recon_sh24-to-rbm_pth.pkl")?,
        Default::default(),
    )?;
    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
    let agws_parts = Agws::<1, 1, K48, K24>::builder()
        .gmt(gmtb.clone())
        .sh24(ShackHartmannBuilder::sh24().use_calibration_src())
        .sh24_calibration(sh24_recon)
        .sh48(ShackHartmannBuilder::sh48().use_calibration_src())
        .sh48_calibration(reconstructor)
        .parts()?;

    let omb = OpticalModel::<NoSensor>::builder()
        .gmt(gmtb)
        .source(AgwsGuideStar::sh48());
    let om = OpticalModelBuilder::<WaveSensorBuilder>::from(&omb).build()?;

    Ok((agws_parts, om))
}

fn sh24_estimation(
    optical_state: &mut OpticalState,
    AgwsParts {
        sh24, sh24_kernel, ..
    }: &mut AgwsParts<1, 1, K48, K24>,
) -> std::result::Result<(), Box<dyn Error>> {
    // SH24 M2 Rxy correction
    <_ as Read<OpticsState>>::read(sh24, Data::new(optical_state.clone()));
    sh24.update();
    let data = <_ as Write<KernelFrame<K24>>>::write(sh24).unwrap();

    <_ as TryRead<KernelFrame<K24>>>::try_read(sh24_kernel, data)?;
    sh24_kernel.try_update()?;
    let data = <_ as TryWrite<M2RigidBodyMotions>>::try_write(sh24_kernel)?.unwrap();

    optical_state.m2_as_mut().map(|m2| {
        m2.iter_mut()
            .zip(data.chunks(6))
            .for_each(|(segment, rbms)| {
                segment.map(|segment| {
                    *segment = segment.clone() - SegmentState::rbms(rbms);
                });
            })
    });
    Ok(())
}
fn estimation(
    optical_state: &OpticalState,
    AgwsParts {
        sh48,
        sh24,
        sh24_kernel,
        sh48_kernel,
    }: &mut AgwsParts<1, 1, K48, K24>,
) -> std::result::Result<Data<Estimate>, Box<dyn Error>> {
    let mut optical_state = optical_state.clone();
    // SH24 M2 Rxy correction
    <_ as Read<OpticsState>>::read(sh24, Data::new(optical_state.clone()));
    sh24.update();
    let data = <_ as Write<KernelFrame<K24>>>::write(sh24).unwrap();

    <_ as TryRead<KernelFrame<K24>>>::try_read(sh24_kernel, data)?;
    sh24_kernel.try_update()?;
    let data = <_ as TryWrite<M2RigidBodyMotions>>::try_write(sh24_kernel)?.unwrap();

    optical_state.m2_as_mut().map(|m2| {
        m2.iter_mut()
            .zip(data.chunks(6))
            .for_each(|(segment, rbms)| {
                segment.map(|segment| {
                    *segment = segment.clone() - SegmentState::rbms(rbms);
                });
            })
    });

    println!("M2 RBMS w/ SH24 Rxy");
    let m2_rbms = optical_state
        .m2_as_mut()
        .and_then(|m2| m2.into_rbms())
        .unwrap();
    print_rbms::<2>(m2_rbms.chunks(6), Option::<iter::Empty<&[f64]>>::None);

    // SH48 M2 Txy estimation
    <_ as Read<OpticsState>>::read(sh48, Data::new(optical_state));
    sh48.update();
    let data = <_ as Write<KernelFrame<K48>>>::write(sh48).unwrap();

    <_ as TryRead<KernelFrame<K48>>>::try_read(sh48_kernel, data)?;
    sh48_kernel.try_update()?;
    let data = <_ as TryWrite<Estimate>>::try_write(sh48_kernel)?.unwrap();

    Ok(data)
}

fn gmt_optical_state() -> OpticalState {
    let mut rbms = vec![0f64; 6];
    rbms[0] = 1e-6; // Tx
    // rbms[1] = 1e-6; // Ty
    // rbms[2] = 1e-6; // Tz
    // OpticalState::m2(MirrorState::from(SegmentState::rbms(rbms)))
    OpticalState::new(
        MirrorState::from(SegmentState::rbms(rbms)),
        MirrorState::rbms(),
    )
    // OpticalState::new(
    //     MirrorState::from(
    //         SegmentState::modes(vec![0f64; config::m1::segment::N_MODE])
    //             .set_mode(config::m1::segment::N_MODE-1, 1e-7),
    //     ),
    //     MirrorState::rbms(),
    // )
}

fn residual_opd(
    suffix: &str,
    optical_state: OpticalState,
) -> std::result::Result<(), Box<dyn Error>> {
   let recon = Sh48Calibration::<Stack, M2Txy>::new()?
        .m1_modes(config::m1::segment::MODES, config::m1::segment::N_MODE)?
        .recon()?;
    let (mut agws_parts, mut sh48_opds) = agws(recon)?;

    // let optical_state = gmt_optical_state();
    let data = estimation(&optical_state, &mut agws_parts)?;

    println!("M2 RBMS Txy(z) estimation with SH48");
    let rbms = data
        .chunks(6 + config::m1::segment::N_MODE)
        .map(|x| &x[..6]);
    let modes = data
        .chunks(6 + config::m1::segment::N_MODE)
        .map(|x| &x[6..]);
    let m2: MirrorState = rbms.map(|rbms| SegmentState::rbms(rbms) * -1f64).collect();
    let m1: MirrorState = modes
        .map(|modes| SegmentState::modes(modes) * -1f64)
        .collect();

    let mut final_optical_state = optical_state + OpticalState::new(m1, m2);
    sh24_estimation(&mut final_optical_state, &mut agws_parts)?;

    // AGWS SH48 OPDs
    <_ as Read<OpticsState>>::read(&mut sh48_opds, Data::new(final_optical_state.clone()));
    sh48_opds.update();
    let opds = <_ as Write<Wavefront>>::write(&mut sh48_opds).unwrap();

    let mut sh48_opds = gif::Frame::<f64>::new(format!("sh48_opds_{suffix}.png"), 512);
    <_ as Read<Wavefront>>::read(&mut sh48_opds, opds);
    sh48_opds.update();

    // On-axis WFE RMS
    let gmtb = Gmt::builder().m1(config::m1::segment::MODES, config::m1::segment::N_MODE);
    let mut on_axis = OpticalModel::<NoSensor>::builder()
        .gmt(gmtb.clone())
        .build()?;
    <_ as Read<OpticsState>>::read(&mut on_axis, Data::new(final_optical_state.clone()));
    on_axis.update();
    <_ as Write<WfeRms<-9>>>::write(&mut on_axis)
        .map(|on_axis_wfe_rms| println!("On-axis WFE RMS: {:.0}nm", on_axis_wfe_rms[0]));

    let data = final_optical_state
        .m2_as_ref()
        .unwrap()
        .into_rbms()
        .unwrap();
    let rbms = data.chunks(6);
    let data: Vec<_> = final_optical_state
        .m1_as_ref()
        .unwrap()
        .modes_into_iter()
        .flat_map(|data| data.unwrap().to_vec())
        .collect();
    let modes = data.chunks(config::m1::segment::N_MODE);
    print_rbms::<2>(rbms, Some(modes));
    let data = final_optical_state
        .m1_as_ref()
        .unwrap()
        .into_rbms()
        .unwrap();
    let rbms = data.chunks(6);
    print_rbms::<1>(rbms, None::<iter::Empty<&[f64]>>);
    Ok(())
}

#[test]
fn m1_tx_residuals() -> std::result::Result<(), Box<dyn Error>> {
    residual_opd(
        "M1-Tx",
        OpticalState::new(
            MirrorState::from(SegmentState::rbms({
                let mut rbms = vec![0f64; 6];
                rbms[0] = 1e-6;
                rbms
            })),
            MirrorState::rbms(),
        ),
    )
}
#[test]
fn m1_ty_residuals() -> std::result::Result<(), Box<dyn Error>> {
    residual_opd(
        "M1-Ty",
        OpticalState::new(
            MirrorState::from(SegmentState::rbms({
                let mut rbms = vec![0f64; 6];
                rbms[1] = 1e-6;
                rbms
            })),
            MirrorState::rbms(),
        ),
    )
}
#[test]
fn m1_rx_residuals() -> std::result::Result<(), Box<dyn Error>> {
    residual_opd(
        "M1-Rx",
        OpticalState::new(
            MirrorState::from(SegmentState::rbms({
                let mut rbms = vec![0f64; 6];
                rbms[3] = 1e-6;
                rbms
            })),
            MirrorState::rbms(),
        ),
    )
}
#[test]
fn m1_ry_residuals() -> std::result::Result<(), Box<dyn Error>> {
    residual_opd(
        "M1-Ry",
        OpticalState::new(
            MirrorState::from(SegmentState::rbms({
                let mut rbms = vec![0f64; 6];
                rbms[4] = 1e-6;
                rbms
            })),
            MirrorState::rbms(),
        ),
    )
}
