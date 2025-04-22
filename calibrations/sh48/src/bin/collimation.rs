/*!
#GMT Segments Collimation

Evaluation of the collimation of each GMT telescope starting from a phased M1 assembly.

The collimation is performed using 3 SH48 WFSs with M2 TT in closed-loop with SH24 and
with some M1 bending modes applied.

*/

use std::fs::File;

use gmt_dos_clients_crseo::{
    calibration::{Calib, ClosedLoopCalib, Reconstructor}, crseo::{FromBuilder, Gmt, Source}, sensors::{builders::WaveSensorBuilder, Camera, NoSensor, WaveSensor}, OpticalModel, OpticalModelBuilder
};
use gmt_dos_clients_io::{
    gmt_m2::M2RigidBodyMotions,
    optics::{M1Modes, SegmentWfeRms, SensorData, Wavefront, WfeRms},
};
use gmt_dos_systems_agws::{
    agws::{sh24::Sh24TT, sh48::Sh48},
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::{Kernel, KernelFrame},
};
use interface::{Data, Read, Write};
use skyangle::Conversion;
use gmt_dos_clients::gif;
use gmt_ns_im::{MergeReconstructor, SplitEstimate};

fn main() -> anyhow::Result<()> {
    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let mut kern48 = Kernel::<Sh48<1>>::try_from(&sh48)?;
    let mut om48 = OpticalModelBuilder::<_>::from(sh48)
        .gmt(Gmt::builder().m1(
            gmt_ns_im::config::m1::segment::MODES,
            gmt_ns_im::config::m1::segment::N_MODE,
        ))
        .build()?;

    let src = Source::builder().size(4).zenith_azimuth(vec![0.,6f32.from_arcmin(),7f32.from_arcmin(),8f32.from_arcmin()], vec![0.,0.,120f32.to_radians(),240f32.to_radians()]);
    let mut score= OpticalModelBuilder::<WaveSensorBuilder>::from(&OpticalModel::<NoSensor>::builder().
        gmt(Gmt::builder().m1(
            gmt_ns_im::config::m1::segment::MODES,
            gmt_ns_im::config::m1::segment::N_MODE,
        ))
        .source(src.clone())).build()?;
    // Perturbations
    let mut m2_rbm = vec![vec![0f64; 6]; 7];
    m2_rbm[0][0] = 1e-6;
    m2_rbm[1][1] = 1e-6;
    m2_rbm[2][0] = 1e-6;
    m2_rbm[3][1] = 1e-6;
    m2_rbm[4][0] = 1e-6;
    m2_rbm[5][1] = 1e-6;
    m2_rbm[6][1] = 1e-6;
    // open_loop(m2_rbm ,&mut om48, &mut kern48, &mut score)?;
    // closed_loop(m2_rbm ,&mut om48, &mut kern48, &mut score)?;
    merged_closed_loop(m2_rbm ,&mut om48, &mut kern48, &mut score)?;
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
    println!("Segment WFS RMS:");
    segment_wfe_rms.chunks(7).enumerate().for_each(|(i,wfe)|
    println!("#{:2}: {:5.0?}nm",i+1, wfe)
    );
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
    println!("{score}");
    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone().into();
        score;
        SegmentWfeRms<-9>: segment_wfe_rms
    );
    println!("Segment WFS RMS:");
    segment_wfe_rms.chunks(7).enumerate().for_each(|(i,wfe)|
    println!("#{:2}: {:5.0?}nm",i+1, wfe)
    );

    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone().into();
        &mut om24;
        KernelFrame<Sh24TT<1>>;
        &mut kern24;
        M2RigidBodyMotions: m2_rbm_tt);
    
    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone()-m2_rbm_tt.clone();
        score;
        SegmentWfeRms<-9>: segment_wfe_rms
    );
    println!("Segment WFS RMS:");
    segment_wfe_rms.chunks(7).enumerate().for_each(|(i,wfe)|
    println!("#{:2}: {:5.0?}nm",i+1, wfe)
    );
    let mut wavefronts: gif::Frame<f64> = gif::Frame::new("wavefronts.png", 512);
    interface::chain!(
        score;
        Wavefront;
        &mut wavefronts 
    );
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
fn merged_closed_loop(m2_rbm: Vec<Vec<f64>>,om48: &mut OpticalModel<Camera>,kern48: &mut Kernel<Sh48<1>>, score: &mut OpticalModel<WaveSensor>) -> anyhow::Result<()> {
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

    // let mut sh48_m2_rbm_recon: Reconstructor<_, ClosedLoopCalib> = serde_pickle::from_reader(
    //     File::open("closed_loop_recon_sh48-to-m2-rbm.pkl")?,
    //     Default::default(),
    // )?;
    let mut sh48_m2_rbm_recon = MergeReconstructor::new("closed_loop_recon_sh48-to-m2-rbm.pkl","closed_loop_recon_sh48-to-m1-bm.pkl",None)?;
    // sh48_m2_rbm_recon.truncated_pseudoinverse(vec![1, 1, 1, 1, 1, 1, 0]);
    println!("CLOSED LOOP SH48 M2 RBM {sh48_m2_rbm_recon}");

    let mut m1_bm = vec![vec![0f64; 27]; 7];
    m1_bm[0][0] = 1e-6;
    m1_bm[1][1] = 1e-6;
    m1_bm[2][2] = 1e-6;
    m1_bm[3][3] = 1e-6;
    m1_bm[4][4] = 1e-6;
    m1_bm[5][5] = 1e-6;
    m1_bm[6][6] = 1e-6;
    let m1_bm = m1_bm.into_iter().flatten().collect::<Vec<_>>();

    <_ as Read<M1Modes>>::read(&mut om24,m1_bm.clone().into());
    <_ as Read<M1Modes>>::read(om48,m1_bm.clone().into());
    <_ as Read<M1Modes>>::read(score,m1_bm.clone().into());
    
    let m2_rbm = m2_rbm.into_iter().flatten().collect::<Vec<_>>();
    println!("{score}");
    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone().into();
        score;
        WfeRms<-9>: wfe_rms
    );
    println!("WFS RMS: {:5.0?}nm", &*wfe_rms);

    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone().into();
        &mut om24;
        KernelFrame<Sh24TT<1>>;
        &mut kern24;
        M2RigidBodyMotions: m2_rbm_tt);
    
    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone()-m2_rbm_tt.clone();
        score;
        WfeRms<-9>: wfe_rms
    );
    println!("WFS RMS: {:5.0?}nm", &*wfe_rms);
    let mut wavefronts: gif::Frame<f64> = gif::Frame::new("wavefronts.png", 512);
    interface::chain!(
        score;
        Wavefront;
        &mut wavefronts 
    );
    interface::chain!(
        M2RigidBodyMotions: m2_rbm.clone()-m2_rbm_tt.clone();
        om48;
        KernelFrame<Sh48<1>>;
        kern48;
        SensorData; 
        &mut sh48_m2_rbm_recon;
        SplitEstimate<0>: m2_rbm_e);
    println!("M2 RBM Estimates:");
    m2_rbm_e
        .chunks(6)
        .map(|x| x.iter().map(|&x| x * 1e6).collect::<Vec<_>>())
        .for_each(|x| println!("{:6.3?}", x));

let m1_bm_e =   <_ as Write<SplitEstimate<1>>>::write(&mut sh48_m2_rbm_recon).unwrap();
    println!("M1 BM Estimates:");
    m1_bm_e
        .chunks(27)
        .map(|x| x.iter().take(7).map(|&x| x * 1e6).collect::<Vec<_>>())
        .for_each(|x| println!("{:6.3?}", x));

    <_ as Read<M1Modes>>::read(score,m1_bm.clone().into_iter().zip(m1_bm_e.iter()).map(|(x,y)|x-*y).collect::<Vec<f64>>().into());
    interface::chain!(
        M2RigidBodyMotions: Data::new(m2_rbm.clone()) - &*m2_rbm_e ;
        score;
        WfeRms<-9>: wfe_rms
    );
    println!("WFS RMS: {:5.0?}nm", &*wfe_rms);
    let mut wavefronts: gif::Frame<f64> = gif::Frame::new("residual_wavefronts.png", 512);
    interface::chain!(
        score;
        Wavefront;
        &mut wavefronts 
    );
    
    Ok(())
}
