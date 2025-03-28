use std::fs::File;

use gmt_dos_clients_crseo::{
    OpticalModelBuilder,
    calibration::{Calibration, CalibrationMode},
    centroiding::CentroidsProcessing,
    crseo::{
        FromBuilder, Gmt,
        gmt::{GmtM1, GmtM2},
    },
};
use gmt_dos_systems_agws::builder::shack_hartmann::ShackHartmannBuilder;
use skyangle::Conversion;

fn main() -> anyhow::Result<()> {
    let sh48 = ShackHartmannBuilder::<1>::sh48().use_calibration_src();
    let omb = OpticalModelBuilder::<_>::from(sh48)
        .gmt(Gmt::builder().m1(gmt_ns_im::config::m1::segment::MODES, gmt_ns_im::config::m1::segment::N_MODE));
    dbg!(&omb);

    // Calibration of M1 RBMs with AGWS SH48
    let mut recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
        &(omb.clone().into()),
        CalibrationMode::RBM([
            Some(1e-6),
            Some(1e-6),
            Some(1e-6),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
        ]),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("recon_sh48-to-m1-rbm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    // Calibration of M1 BMs with AGWS SH48
    let mut recon = <CentroidsProcessing as Calibration<GmtM1>>::calibrate(
        &(omb.clone().into()),
        CalibrationMode::modes(gmt_ns_im::config::m1::segment::N_MODE, 1e-6),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("recon_sh48-to-m1-bm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    // Calibration of M2 RBMs with AGWS SH48
    let mut recon = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(omb.into()),
        CalibrationMode::RBM([
            Some(1e-6),
            Some(1e-6),
            Some(1e-6),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
        ]),
    )?;
    recon.pseudoinverse();
    println!("{recon}");
    let mut file = File::create("recon_sh48-to-m2-rbm.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    Ok(())
}
