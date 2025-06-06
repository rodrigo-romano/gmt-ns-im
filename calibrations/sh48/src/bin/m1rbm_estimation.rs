use std::fs::File;

use gmt_dos_clients_crseo::{
    OpticalModel, OpticalModelBuilder,
    calibration::{CalibrationMode, ClosedLoopCalib, Reconstructor},
    crseo::{FromBuilder, Gmt, Source},
    sensors::{NoSensor, builders::WaveSensorBuilder},
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions,
    gmt_m2::M2RigidBodyMotions,
    optics::{SegmentWfeRms, SensorData},
};
use gmt_dos_systems_agws::{
    agws::{sh24::Sh24TT, sh48::Sh48},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::{Kernel, KernelFrame},
};
use interface::filing::Filing;
use skyangle::Conversion;

fn main() -> anyhow::Result<()> {
    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let mut kern48 = Kernel::<Sh48<1>>::try_from(&sh48)?;
    let mut om48 = OpticalModelBuilder::<_>::from(sh48)
        .gmt(Gmt::builder().m1(
            gmt_ns_im::config::m1::segment::MODES,
            gmt_ns_im::config::m1::segment::N_MODE,
        ))
        .build()?;

    let src = Source::builder().size(4).zenith_azimuth(
        vec![
            0.,
            6f32.from_arcmin(),
            7f32.from_arcmin(),
            8f32.from_arcmin(),
        ],
        vec![0., 0., 120f32.to_radians(), 240f32.to_radians()],
    );
    let mut score = OpticalModelBuilder::<WaveSensorBuilder>::from(
        &OpticalModel::<NoSensor>::builder()
            .gmt(Gmt::builder().m1(
                gmt_ns_im::config::m1::segment::MODES,
                gmt_ns_im::config::m1::segment::N_MODE,
            ))
            .source(src.clone()),
    )
    .build()?;

    let recon: Reconstructor = serde_pickle::from_reader(
        File::open("../sh24/recon_sh24-to-rbm_pth.pkl")?,
        Default::default(),
    )?;
    println!("SH48 M2 RBM {recon}");
    let sh24 = ShackHartmannBuilder::<1>::sh24()
        .use_calibration_src()
        .reconstructor(recon);

    let mut kern24 = Kernel::<Sh24TT<1>>::try_from(&sh24)?;
    let mut om24 = OpticalModelBuilder::<_>::from(sh24)
        .gmt(Gmt::builder().m1(
            gmt_ns_im::config::m1::segment::MODES,
            gmt_ns_im::config::m1::segment::N_MODE,
        ))
        .build()?;

    // let mut m1_recon: Reconstructor<CalibrationMode, ClosedLoopCalib> = serde_pickle::from_reader(
    //     File::open("../sh48/closed_loop_recon_sh48-to-m1-rbm.pkl")?,
    //     Default::default(),
    // )?;
    let mut e_1_48: Reconstructor = Reconstructor::from_path("e_1_48.pkl")?;
    // println!("SH48 M1 RBM {m1_recon}");

    let sid = 0;
    let j = 3;
    let mut rbm = vec![vec![0f64; 6]; 7];
    rbm[sid][j] = 1e-6;
    let rbm = rbm.into_iter().flatten().collect::<Vec<_>>();
    interface::chain!(
        M1RigidBodyMotions: rbm.as_slice();
        &mut om24;
        KernelFrame<Sh24TT<1>>;
        &mut kern24;
        M2RigidBodyMotions: m2_rbm_tt
    );
    interface::chain!(
        M1RigidBodyMotions: rbm.as_slice(),
        M2RigidBodyMotions: -m2_rbm_tt.clone();
        &mut om48;
        KernelFrame<Sh48<1>>;
        &mut kern48;
        SensorData;
        &mut e_1_48;
        // &mut m1_recon;
        M1RigidBodyMotions: m1_rbm_e
    );
    m1_rbm_e
        .chunks(6)
        // .skip(sid)
        // .take(1)
        .map(|x| x.iter().map(|&x| x * 1e9).collect::<Vec<_>>())
        .for_each(|x| println!("{:+06.0?}", x));

    /* // M1 perturbations
    let mut rbm = vec![vec![0f64; 6]; 7];
    rbm[0][4] = 1e-7;

    let rbm = rbm.into_iter().flatten().collect::<Vec<_>>();
    println!("{score}");
    interface::chain!(
        M1RigidBodyMotions: rbm.as_slice();
        &mut score;
        SegmentWfeRms<-9>: segment_wfe_rms
    );
    println!("Segment WFS RMS:");
    segment_wfe_rms
        .chunks(7)
        .enumerate()
        .for_each(|(i, wfe)| println!("#{:2}: {:5.0?}nm", i + 1, wfe));

    interface::chain!(
        M1RigidBodyMotions: rbm.as_slice();
        &mut om24;
        KernelFrame<Sh24TT<1>>;
        &mut kern24;
        M2RigidBodyMotions: m2_rbm_tt);
    println!("M2 TT Estimates:");
    m2_rbm_tt
        .chunks(6)
        .map(|x| x.iter().map(|&x| x * 1e9).collect::<Vec<_>>())
        .for_each(|x| println!("{:6.0?}", x));
    interface::chain!(
        M1RigidBodyMotions: rbm.as_slice(),
        M2RigidBodyMotions: -m2_rbm_tt.clone();
        &mut score;
        SegmentWfeRms<-9>: segment_wfe_rms
    );
    println!("Segment WFS RMS:");
    segment_wfe_rms
        .chunks(7)
        .enumerate()
        .for_each(|(i, wfe)| println!("#{:2}: {:5.0?}nm", i + 1, wfe));

    interface::chain!(
        M1RigidBodyMotions: rbm.as_slice(),
        M2RigidBodyMotions: -m2_rbm_tt.clone();
        &mut om48;
        KernelFrame<Sh48<1>>;
        &mut kern48;
        SensorData;
        &mut m1_recon;
        M1RigidBodyMotions: m1_rbm_e
    );
    println!("M1 RBM Estimates:");
    m1_rbm_e
        .chunks(6)
        .map(|x| x.iter().map(|&x| x * 1e9).collect::<Vec<_>>())
        .for_each(|x| println!("{:6.0?}", x));
    // interface::chain!(
    //     M1RigidBodyMotions: rbm.clone() - m1_rbm_e.clone(),
    //     // M2RigidBodyMotions: -m2_rbm_tt.clone();
    //     M2RigidBodyMotions: vec![0f64;42];
    //     &mut score;
    //     SegmentWfeRms<-9>: segment_wfe_rms
    // );
    // println!("Segment WFS RMS:");
    // segment_wfe_rms
    //     .chunks(7)
    //     .enumerate()
    //     .for_each(|(i, wfe)| println!("#{:2}: {:5.0?}nm", i + 1, wfe));

    interface::chain!(
        M1RigidBodyMotions: rbm.clone() - m1_rbm_e.clone(),
        M2RigidBodyMotions: -m2_rbm_tt.clone();
        &mut om24;
        KernelFrame<Sh24TT<1>>;
        &mut kern24;
        M2RigidBodyMotions: m2_rbm_tt_1);
    println!("M2 TT Estimates:");
    m2_rbm_tt_1
        .chunks(6)
        .map(|x| x.iter().map(|&x| x * 1e9).collect::<Vec<_>>())
        .for_each(|x| println!("{:6.0?}", x));
    interface::chain!(
        M1RigidBodyMotions: rbm.clone() - m1_rbm_e.clone(),
        M2RigidBodyMotions: -m2_rbm_tt.clone()-m2_rbm_tt_1.clone();
        &mut score;
        SegmentWfeRms<-9>: segment_wfe_rms
    );
    println!("Segment WFS RMS:");
    segment_wfe_rms
        .chunks(7)
        .enumerate()
        .for_each(|(i, wfe)| println!("#{:2}: {:5.0?}nm", i + 1, wfe)); */
    Ok(())
}
