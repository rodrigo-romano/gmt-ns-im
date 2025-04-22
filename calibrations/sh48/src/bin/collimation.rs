/*!
#GMT Segments Collimation

Evaluation of the collimation of each GMT telescope starting from a phased M1 assembly.

The collimation is performed using 3 SH48 WFSs.

*/

use std::fs::File;

use calibrations_sh48::open_loop::m2_clocking;
use gmt_dos_clients_crseo::{
    calibration::{Calib, ClosedLoopCalib, Reconstructor}, crseo::{FromBuilder, Gmt, Source}, sensors::{Camera, WaveSensor}, OpticalModel, OpticalModelBuilder
};
use gmt_dos_clients_io::{
    gmt_m2::M2RigidBodyMotions,
    optics::{SegmentWfeRms, SensorData},
};
use gmt_dos_systems_agws::{
    agws::{sh24::Sh24TT, sh48::Sh48},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::{Kernel, KernelFrame},
};

fn main() -> anyhow::Result<()> {
    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let mut kern48 = Kernel::<Sh48<1>>::try_from(&sh48)?;
    let mut om48 = OpticalModelBuilder::<_>::from(sh48)
        .gmt(Gmt::builder().m1(
            gmt_ns_im::config::m1::segment::MODES,
            gmt_ns_im::config::m1::segment::N_MODE,
        ))
        .build()?;

    let src = Source::builder().zenith_azimuth(vec![0.], vec![0.]);
    let mut score = OpticalModel::<WaveSensor>::builder()
        .gmt(Gmt::builder().m1(
            gmt_ns_im::config::m1::segment::MODES,
            gmt_ns_im::config::m1::segment::N_MODE,
        ))
        .source(src.clone())
        .sensor(
            WaveSensor::builder()
                // .gmt(self.gmt.clone().unwrap_or_default())
                .source(src),
        )
        .build()?;
    // Perturbations
    let mut m2_rbm = vec![vec![0f64; 6]; 7];
    m2_rbm[0][0] = 1e-6;
    m2_rbm[1][1] = 1e-6;
    m2_rbm[2][2] = 1e-6;
    // m2_rbm[3][3] = 1e-6;
    // m2_rbm[4][4] = 1e-6;
    m2_rbm[5][5] = 1e-6;
    m2_rbm[6][2] = 1e-6;
    #[cfg(feature="open_loop")]
    open_loop(m2_rbm ,&mut om48, &mut kern48, &mut score)?;
    #[cfg(feature="closed_loop")]
    closed_loop(m2_rbm ,&mut om48, &mut kern48, &mut score)?;
    // m2_clocking()?;
    Ok(())
}
fn open_loop(m2_rbm: Vec<Vec<f64>>,om48: &mut OpticalModel<Camera>,kern48: &mut Kernel<Sh48<1>>, score: &mut OpticalModel<WaveSensor>) -> anyhow::Result<()> {
    let mut sh48_m2_rbm_recon: Reconstructor<_, Calib> = serde_pickle::from_reader(
        File::open("open_loop_recon_sh48-to-m2-rbm.pkl")?,
        Default::default(),
    )?;
    // sh48_m2_rbm_recon.truncated_pseudoinverse(vec![1, 1, 1, 1, 1, 1, 0]);
    println!("OPEN LOOP SH48 M2 RBM {sh48_m2_rbm_recon}");

    let m2_rbm = m2_rbm.into_iter().flatten().collect::<Vec<_>>();
    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone().into();
        score;
        SegmentWfeRms<-9>: segment_wfe_rms
    );
    println!("Segment WFS RMS: {:5.0?}nm", &*segment_wfe_rms);
    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone().into();
        om48;
        KernelFrame<Sh48<1>>;
        kern48;
        SensorData; 
        &mut sh48_m2_rbm_recon;
        M2RigidBodyMotions: m2_rbm_e);
    println!("M2 RBM Estimates:");
    m2_rbm_e
        .chunks(6)
        .map(|x| x.iter().map(|&x| x * 1e6).collect::<Vec<_>>())
        .for_each(|x| println!("{:6.3?}", x));
    Ok(())
}
fn closed_loop(m2_rbm: Vec<Vec<f64>>,om48: &mut OpticalModel<Camera>,kern48: &mut Kernel<Sh48<1>>, score: &mut OpticalModel<WaveSensor>) -> anyhow::Result<()> {
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

    let mut sh48_m2_rbm_recon: Reconstructor<_, ClosedLoopCalib> = serde_pickle::from_reader(
        File::open("closed_loop_recon_sh48-to-m2-rbm.pkl")?,
        Default::default(),
    )?;
    // sh48_m2_rbm_recon.truncated_pseudoinverse(vec![1, 1, 1, 1, 1, 1, 0]);
    println!("CLOSED LOOP SH48 M2 RBM {sh48_m2_rbm_recon}");

    let m2_rbm = m2_rbm.into_iter().flatten().collect::<Vec<_>>();
    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone().into();
        score;
        SegmentWfeRms<-9>: segment_wfe_rms
    );
    println!("Segment WFS RMS: {:5.0?}nm", &*segment_wfe_rms);

    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone().into();
        &mut om24;
        KernelFrame<Sh24TT<1>>;
        &mut kern24;
        M2RigidBodyMotions: m2_rbm_tt);
    
    interface::chain!(
        M2RigidBodyMotions: m2_rbm-m2_rbm_tt;
        om48;
        KernelFrame<Sh48<1>>;
        kern48;
        SensorData; 
        &mut sh48_m2_rbm_recon;
        M2RigidBodyMotions: m2_rbm_e);
    println!("M2 RBM Estimates:");
    m2_rbm_e
        .chunks(6)
        .map(|x| x.iter().map(|&x| x * 1e6).collect::<Vec<_>>())
        .for_each(|x| println!("{:6.3?}", x));
    Ok(())
}
