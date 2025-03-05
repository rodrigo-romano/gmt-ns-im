use std::fs::File;

use faer::MatRef;
use gmt_dos_clients_crseo::{
    calibration::{Calibration, CalibrationMode},
    centroiding::CentroidsProcessing,
    crseo::gmt::GmtM2,
};
use gmt_dos_systems_agws::{Agws, builder::AgwsGuideStar};
use matio_rs::MatFile;
use skyangle::Conversion;

fn main() -> anyhow::Result<()> {
    let omb = Agws::<1, 1>::builder()
        .sh24()
        // calibration source with FWHM roughly the size of the seeing
        .source(AgwsGuideStar::sh24().fwhm(12f64));
    // dbg!(&omb);

    // Calibration of M2 Tz,Rx,Ry RBMs with AGWS SH24
    let mut recon = <CentroidsProcessing as Calibration<GmtM2>>::calibrate(
        &(omb.into()),
        CalibrationMode::RBM([
            None,
            None,
            Some(1e-6),
            Some(1f64.from_arcsec()),
            Some(1f64.from_arcsec()),
            None,
        ]),
    )?;
    recon.pseudoinverse();
    println!("{recon}");

    // Segment-wise multiplication of poke matrix pseudo-inverse
    // with [Tz,Rx,Ry]->(PZT actuators) matrix
    let matfile = MatFile::load("rbm_2_pzt.mat")?;
    recon.pinv().enumerate().for_each(|(i, pinv)| {
        let var: Vec<f64> = matfile.var(format!("var{i}")).unwrap();
        let mat = MatRef::from_row_major_slice(&var, 3, 3);
        pinv.transform(|x| mat * x);
    });

    let mut file = File::create("recon_sh24.pkl")?;
    serde_pickle::to_writer(&mut file, &recon, Default::default())?;

    Ok(())
}
