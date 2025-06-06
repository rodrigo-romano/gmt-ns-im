use std::fs::File;

use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    calibration::{Calibration, CalibrationMode, ClosedLoopCalib, MirrorMode, Reconstructor},
    centroiding::CentroidsProcessing,
    crseo::{
        FromBuilder, Gmt,
        gmt::{GmtM1, GmtM2},
    },
};
use gmt_dos_clients_io::{
    gmt_m1::M1RigidBodyMotions, gmt_m2::M2RigidBodyMotions, optics::SensorData,
};
use gmt_dos_systems_agws::{
    agws::{
        sh24::{Sh24, Sh24TT},
        sh48::Sh48,
    },
    builder::shack_hartmann::ShackHartmannBuilder,
    kernels::{Kernel, KernelFrame},
};
use interface::filing::Filing;

fn main() -> anyhow::Result<()> {
    let sh24 = ShackHartmannBuilder::<1>::sh24().use_calibration_src();
    let omb24 = OpticalModelBuilder::<_>::from(sh24.clone());

    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let omb48 = OpticalModelBuilder::<_>::from(sh48.clone()).gmt(Gmt::builder().m1(
        gmt_ns_im::config::m1::segment::MODES,
        gmt_ns_im::config::m1::segment::N_MODE,
    ));

    // Calibration of M2 Rx,Ry RBMs with AGWS SH24
    let mut d_2_24 = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(omb24.clone().into()),
        CalibrationMode::r_xy(1e-6),
    )?;
    d_2_24.pseudoinverse();
    d_2_24.to_path("d_2_24.pkl")?;
    println!("{d_2_24}");

    // Calibration of M1 RBMs with AGWS SH24
    let m1_rbm = MirrorMode::from(CalibrationMode::RBM([
        None,
        Some(1e-6),
        Some(1e-6),
        Some(1e-6),
        Some(1e-6),
        None,
    ]))
    .update((7, CalibrationMode::empty_rbm()));
    let d_1_24 = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
        &(omb24.clone().into()),
        m1_rbm.clone(),
    )?;
    println!("{d_1_24}");

    // Calibration of M2 Rx,Ry RBMs with AGWS SH48
    let d_2_48 = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(omb48.clone().into()),
        CalibrationMode::r_xy(1e-6),
    )?;
    println!("{d_2_48}");

    // Calibration of M1 RBMs with AGWS SH24
    let d_1_48 =
        <CentroidsProcessing as Calibration<GmtM1>>::calibrate(&(omb48.clone().into()), m1_rbm)?;
    println!("{d_1_48}");

    let mut e_1_48 = d_1_48.clone();
    e_1_48 -= (&d_2_48 * d_2_24.pinv_as_ref()) * &d_1_24;
    // e_1_48.truncated_pseudoinverse(vec![1; 7]);
    e_1_48.pseudoinverse();
    println!("{e_1_48}");
    e_1_48.to_path("e_1_48.pkl")?;

    e_1_48
        .calib_pinv()
        .map(|(d, e)| e * d)
        .enumerate()
        .for_each(|(i, m)| println!("S{}:{:+06.0?}", i + 1, m * 1e3));

    let sh24 = sh24.reconstructor(d_2_24);
    let mut kern24 = Kernel::<Sh24TT<1>>::try_from(&sh24)?;
    let mut om24 = omb24.build()?;
    let mut kern48 = Kernel::<Sh48<1>>::try_from(&sh48)?;
    let mut om48 = omb48.build()?;
    let mut m1_recon: Reconstructor<CalibrationMode, ClosedLoopCalib> = serde_pickle::from_reader(
        File::open("../sh48/closed_loop_recon_sh48-to-m1-rbm.pkl")?,
        Default::default(),
    )?;
    let mut t = vec![1; 7];
    t[6] = 0;
    m1_recon.truncated_pseudoinverse(t);
    println!("SH48 M1 RBM {m1_recon}");

    println!("M1 RBM Estimates:");
    for sid in 0..7 {
        println!("S{}:[", sid + 1);
        for j in 0..6 {
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
                .skip(sid)
                .take(1)
                .map(|x| x.iter().map(|&x| x * 1e9).collect::<Vec<_>>())
                .for_each(|x| println!("{:+06.0?}", x));
        }
        println!("]");
    }
    Ok(())
}
