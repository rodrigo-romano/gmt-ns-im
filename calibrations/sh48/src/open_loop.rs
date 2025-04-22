use std::fs::File;

use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    calibration::{Calibration, CalibrationMode, MirrorMode},
    centroiding::CentroidsProcessing,
    crseo::{
        FromBuilder, Gmt,
        gmt::{GmtM1, GmtM2},
    },
};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;

use crate::SH48CalibrationError;

pub fn m1_rbm() -> Result<(), SH48CalibrationError> {
    println!("SH48 OPEN-LOOP CALIBRATION OF M1 RBM");

    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let omb48 = OpticalModelBuilder::<_>::from(sh48).gmt(Gmt::builder().m1(
        gmt_ns_im::config::m1::segment::MODES,
        gmt_ns_im::config::m1::segment::N_MODE,
    ));

    let mut c7 = [Some(1e-6); 6];
    c7[5] = None;
    let c7 = CalibrationMode::RBM(c7);
    let c = MirrorMode::from(CalibrationMode::rbm(1e-6)).update((7, c7));
    let mut recon =
        <CentroidsProcessing as Calibration<GmtM1>>::calibrate(&(omb48.clone().into()), c)?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("open_loop_recon_sh48-to-m1-rbm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;
    Ok(())
}
pub fn m1_rxy() -> Result<(), SH48CalibrationError> {
    println!("SH48 OPEN-LOOP CALIBRATION OF M1 RXY");

    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let omb48 = OpticalModelBuilder::<_>::from(sh48).gmt(Gmt::builder().m1(
        gmt_ns_im::config::m1::segment::MODES,
        gmt_ns_im::config::m1::segment::N_MODE,
    ));

    let mut recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
        &(omb48.clone().into()),
        CalibrationMode::r_xy(1e-6),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("open_loop_recon_sh48-to-m1-rxy.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;
    Ok(())
}
pub fn m1_bm() -> Result<(), SH48CalibrationError> {
    println!("SH48 OPEN-LOOP CALIBRATION OF M1 BM");

    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let omb48 = OpticalModelBuilder::<_>::from(sh48).gmt(Gmt::builder().m1(
        gmt_ns_im::config::m1::segment::MODES,
        gmt_ns_im::config::m1::segment::N_MODE,
    ));

    let mut recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
        &(omb48.clone().into()),
        CalibrationMode::modes(gmt_ns_im::config::m1::segment::N_MODE, 1e-6),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("open_loop_recon_sh48-to-m1-bm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;
    Ok(())
}
pub fn m2_rbm() -> Result<(), SH48CalibrationError> {
    println!("SH48 OPEN-LOOP CALIBRATION OF M2 RBM");

    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let omb48 = OpticalModelBuilder::<_>::from(sh48).gmt(Gmt::builder().m1(
        gmt_ns_im::config::m1::segment::MODES,
        gmt_ns_im::config::m1::segment::N_MODE,
    ));

    let mut c7 = [Some(1e-6); 6];
    c7[5] = None;
    let c7 = CalibrationMode::RBM(c7);
    // let c = MirrorMode::from(CalibrationMode::rbm(1e-6)).update((7, c7));
    let c = MirrorMode::from(c7);
    let mut recon =
        <CentroidsProcessing as Calibration<GmtM2>>::calibrate(&(omb48.clone().into()), c)?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("open_loop_recon_sh48-to-m2-rbm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;
    Ok(())
}
pub fn m2_clocking() -> Result<(), SH48CalibrationError> {
    let rbm = geotrans::Mirror::<geotrans::M2>::clocking_2_rigidbodymotions(1e-6);
    let mut file = File::create("m2_clocking_rbms.pkl")?;
    serde_pickle::to_writer(&mut file, &rbm, Default::default())?;
    Ok(())
}
